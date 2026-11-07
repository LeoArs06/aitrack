#include "../../Client/src/hybrid_qt/ImuWorker.h"
#include "../../Client/src/hybrid_qt/HostClock.h"
#include "../../Client/src/model/Config.h"
#include <QCoreApplication>
#include <QDir>
#include <QTemporaryDir>
#include <QThread>
#include <QTimer>
#include <QUdpSocket>
#include <iostream>
#include <cstring>
#include <cmath>

// Real QThread/event-loop/UDP integration without a camera or serial hardware.
// Checks remain enabled in Release.
int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);
    QTemporaryDir temporary;
    if (!temporary.isValid() || !QDir::setCurrent(temporary.path())) return 2;
    int failures = 0;
    auto check = [&](bool ok, const char* message) {
        if (!ok) { ++failures; std::cerr << "FAIL: " << message << '\n'; }
    };
    {
        ConfigMgr prefs("prefs.ini");
        auto c = prefs.getConfig();
        check(!c.hybrid_mode && c.imu_port == "Auto" && c.optical_strength == 50 &&
            c.imu_calibration.empty() && !c.hybrid_log, "preference defaults");
        c.hybrid_mode = true; c.imu_port = "COM42"; c.optical_strength = 83;
        c.hybrid_log = true; c.imu_calibration = hybrid::CalibrationResult{}.serialize();
        prefs.updateConfig(c);
        const auto d = prefs.getConfig();
        check(d.hybrid_mode && d.imu_port == c.imu_port && d.optical_strength == 83 &&
            d.imu_calibration == c.imu_calibration && d.hybrid_log, "preference round trip");
    }
    qputenv("AITRACK_IMU_SIM", "1");
    QUdpSocket receiver, second;
    check(receiver.bind(QHostAddress(QHostAddress::LocalHost), 0), "bind UDP receiver");
    check(second.bind(QHostAddress(QHostAddress::LocalHost), 0), "bind reconfigured receiver");
    int packets = 0, secondPackets = 0;
    double last[6] = {};
    auto drain = [&](QUdpSocket& socket, int& count) {
        while (socket.hasPendingDatagrams()) {
            QByteArray bytes; bytes.resize(int(socket.pendingDatagramSize()));
            socket.readDatagram(bytes.data(), bytes.size());
            check(bytes.size() == 48, "48-byte UDP packet");
            if (bytes.size() == 48) {
                std::memcpy(last, bytes.constData(), 48);
                for (double value : last) check(std::isfinite(value), "finite UDP pose");
            }
            ++count;
        }
    };
    QObject::connect(&receiver, &QUdpSocket::readyRead, &app, [&] { drain(receiver, packets); });
    QObject::connect(&second, &QUdpSocket::readyRead, &app, [&] { drain(second, secondPackets); });
    auto* worker = new hybrid_qt::ImuWorker;
    QThread thread;
    worker->moveToThread(&thread);
    QObject::connect(&thread, &QThread::started, worker, &hybrid_qt::ImuWorker::initialize);
    QObject::connect(&thread, &QThread::finished, worker, &QObject::deleteLater);
    QObject::connect(worker, &hybrid_qt::ImuWorker::stopped, &thread, &QThread::quit, Qt::DirectConnection);
    bool sawSim = false, sawCancelled = false, sawFallback = false;
    quint64 ack = 0;
    QObject::connect(worker, &hybrid_qt::ImuWorker::diagnostics, &app,
        [&](QString text, QString progress, bool connected, bool, bool) {
            sawSim |= connected && text.contains("SIM");
            sawCancelled |= progress.contains("cancelled");
            sawFallback |= text.contains("optical only");
        });
    QObject::connect(worker, &hybrid_qt::ImuWorker::outputAcknowledged, &app,
        [&](bool, quint64 generation) { ack = generation; });
    thread.start();
    auto post = [&](auto fn) { QMetaObject::invokeMethod(worker, fn, Qt::QueuedConnection); };
    const int firstPort = receiver.localPort();
    post([=] {
        worker->configure("127.0.0.1", firstPort, 50, {}, true);
        worker->setOutputEnabled(true, 1); worker->requestConnection(true);
    });
    QTimer optical;
    optical.setInterval(33);
    QObject::connect(&optical, &QTimer::timeout, &app, [&] {
        auto now = hybrid_qt::hostNowUs();
        post([=] { worker->addOptical(now, {true, 12, -8, 4, 1.25, -2.5, 45}); });
    });
    optical.start();
    int stoppedCount = 0, bothStoppedCount = 0;
    QTimer::singleShot(1000, &app, [&] {
        check(sawSim && packets > 140, "SIM connected and approximately 200 Hz UDP");
        std::cout << "SIM packets in first second: " << packets << '\n';
        post([=] { worker->calibrate(true); });
    });
    QTimer::singleShot(1200, &app, [&] { post([=] { worker->calibrate(false); worker->requestConnection(false); }); });
    QTimer::singleShot(2200, &app, [&] {
        check(sawCancelled && sawFallback, "calibration cancel and optical fallback");
        const double expected[] = {1.25, -2.5, 45, 12, -8, 4};
        for (int i = 0; i < 6; ++i) check(std::abs(last[i] - expected[i]) < 0.15, "UDP order and cm/degree units");
        post([=] { worker->setOutputEnabled(false, 2); });
    });
    QTimer::singleShot(2300, &app, [&] { check(ack == 2, "disable acknowledged"); stoppedCount = packets; });
    QTimer::singleShot(2500, &app, [&] {
        check(stoppedCount == packets, "no sends after ownership release");
        const int port = second.localPort();
        post([=] {
            worker->configure("invalid-address", port, 100, {}, true);
            worker->configure("127.0.0.1", port, 100, {}, true);
            worker->setOutputEnabled(true, 3);
        });
    });
    QTimer::singleShot(2800, &app, [&] {
        check(secondPackets > 30, "live UDP destination reconfiguration");
        post([=] {
            worker->requestConnection(true);
            worker->setOutputEnabled(false, 4); worker->setOutputEnabled(true, 5);
            worker->setOutputEnabled(false, 6);
        });
    });
    QTimer::singleShot(3000, &app, [&] { check(ack == 6, "ordered rapid ownership changes"); bothStoppedCount = packets + secondPackets; });
    QTimer::singleShot(3300, &app, [&] {
        check(bothStoppedCount == packets + secondPackets, "no sender remains after rapid switching");
        optical.stop(); post([=] { worker->shutdown(); });
    });
    QObject::connect(&thread, &QThread::finished, &app, &QCoreApplication::quit);
    QTimer::singleShot(7000, &app, [&] { ++failures; app.quit(); });
    app.exec();
    std::cerr << "Worker event loop completed\n";
    check(thread.wait(1000), "bounded clean worker shutdown");
    const auto files = QDir("logs").entryList({"hybrid_*.csv"}, QDir::Files);
    check(files.size() == 1, "one CSV log per session");
    if (!files.isEmpty()) {
        QFile log("logs/" + files.front()); log.open(QIODevice::ReadOnly);
        const auto rows = log.readAll().split('\n');
        int imuRows = 0, opticalRows = 0;
        for (const auto& row : rows) {
            if (row.isEmpty()) continue;
            check(row.split(',').size() == 19, "CSV schema matches header");
            imuRows += row.startsWith("imu,"); opticalRows += row.startsWith("optical,");
        }
        check(imuRows > 200 && opticalRows > 60, "raw IMU and optical logging");
    }
    std::cerr << "Qt worker integration: " << (failures ? "FAIL" : "PASS") << " (" << failures << " failures)\n";
    return failures ? 1 : 0;
}
