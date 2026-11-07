#include "../../Client/src/hybrid_qt/ImuWorker.h"
#include "../../Client/src/hybrid_qt/HostClock.h"
#include "../../Client/src/model/Config.h"
#include <QCoreApplication>
#include <QTemporaryDir>
#include <QDir>
#include <QThread>
#include <QTimer>
#include <iostream>

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);
    QTemporaryDir temp;
    QDir::setCurrent(temp.path());
    qputenv("AITRACK_IMU_SIM", "1");
    hybrid::Simulator::Config c;
    c.durationSec = 14; c.hostOffsetUs = 0; c.mcuOffsetUs = 0;
    c.motion = hybrid::Simulator::Motion::Calibration;
    const hybrid::Simulator reference(c);
    auto* worker = new hybrid_qt::ImuWorker;
    QThread thread;
    worker->moveToThread(&thread);
    QObject::connect(&thread, &QThread::started, worker, &hybrid_qt::ImuWorker::initialize);
    QObject::connect(&thread, &QThread::finished, worker, &QObject::deleteLater);
    QObject::connect(worker, &hybrid_qt::ImuWorker::stopped, &thread, &QThread::quit, Qt::DirectConnection);
    int64_t epoch = 0;
    size_t opticalIndex = 0;
    bool saved = false, still = false, motion = false;
    QString lastFailure;
    ConfigMgr prefs("prefs.ini");
    auto post = [&](auto fn) { QMetaObject::invokeMethod(worker, fn, Qt::QueuedConnection); };
    QObject::connect(worker, &hybrid_qt::ImuWorker::diagnostics, &app,
        [&](QString, QString progress, bool connected, bool, bool) {
            if (connected && !epoch) {
                epoch = hybrid_qt::hostNowUs();
            }
            // No button: calibration is learned in the background.
            still |= progress.contains("Learning sensor mounting");
            motion |= progress.contains("Calibrated automatically");
            if (progress.contains("failed") && lastFailure != progress) {
                lastFailure = progress; std::cerr << progress.toStdString() << '\n';
            }
        });
    QObject::connect(worker, &hybrid_qt::ImuWorker::calibrationSaved, &app, [&](QString text) {
        hybrid::CalibrationResult result;
        saved = hybrid::CalibrationResult::deserialize(text.toStdString(), result) && result.quality > 0.5;
        auto config = prefs.getConfig(); config.imu_calibration = text.toStdString(); prefs.updateConfig(config);
        std::cerr << "Qt calibration quality: " << result.quality << ", latency " << result.latencyMs << " ms\n";
        post([=] { worker->shutdown(); });
    });
    thread.start();
    post([=] { worker->configure("127.0.0.1", 4243, 50, {}, false); worker->setOutputEnabled(true, 1); worker->requestConnection(true); });
    QTimer optical;
    optical.setTimerType(Qt::PreciseTimer); optical.setInterval(5);
    QObject::connect(&optical, &QTimer::timeout, &app, [&] {
        if (!epoch) return;
        const auto now = hybrid_qt::hostNowUs();
        const auto& events = reference.optical();
        while (opticalIndex < events.size() && events[opticalIndex].arrivalUs <= now - epoch) {
            const auto event = events[opticalIndex++];
            post([=] { worker->addOptical(epoch + event.arrivalUs, event.pose); });
        }
    });
    optical.start();
    QTimer::singleShot(14500, &app, [&] { post([=] { worker->shutdown(); }); });
    QObject::connect(&thread, &QThread::finished, &app, &QCoreApplication::quit);
    app.exec();
    const bool clean = thread.wait(1000);
    hybrid::CalibrationResult stored;
    const bool persisted = hybrid::CalibrationResult::deserialize(prefs.getConfig().imu_calibration, stored);
    const bool passed = saved && still && motion && clean && persisted;
    std::cerr << "Qt automatic calibration / saved result: " << (passed ? "PASS" : "FAIL") << '\n';
    return passed ? 0 : 1;
}
