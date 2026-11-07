#include "../../Client/src/hybrid_qt/ImuWorker.h"
#include <QCoreApplication>
#include <QThread>
#include <QTimer>
#include <iostream>

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);
    qunsetenv("AITRACK_IMU_SIM");
    QThread thread;
    auto* worker = new hybrid_qt::ImuWorker;
    worker->moveToThread(&thread);
    QObject::connect(&thread, &QThread::started, worker, &hybrid_qt::ImuWorker::initialize);
    QObject::connect(&thread, &QThread::finished, worker, &QObject::deleteLater);
    QObject::connect(worker, &hybrid_qt::ImuWorker::stopped, &thread, &QThread::quit, Qt::DirectConnection);
    QObject::connect(&thread, &QThread::finished, &app, &QCoreApplication::quit);
    int reports = 0;
    bool wrongConnection = false, disconnected = false;
    QObject::connect(worker, &hybrid_qt::ImuWorker::diagnostics, &app,
        [&](QString, QString, bool connected, bool requested, bool) {
            if (requested) { ++reports; wrongConnection |= connected; }
            else disconnected = true;
        });
    thread.start();
    QMetaObject::invokeMethod(worker, [=] {
        worker->setPort("AITRACK_TEST_NONEXISTENT_PORT");
        worker->requestConnection(true);
    }, Qt::QueuedConnection);
    QTimer::singleShot(1400, &app, [=] {
        QMetaObject::invokeMethod(worker, [=] { worker->requestConnection(false); worker->shutdown(); }, Qt::QueuedConnection);
    });
    QTimer::singleShot(4000, &app, &QCoreApplication::quit);
    app.exec();
    const bool ok = thread.wait(1000) && reports >= 4 && !wrongConnection && disconnected;
    std::cerr << "Qt missing-port retry / disconnect / shutdown: " << (ok ? "PASS" : "FAIL") << '\n';
    return ok ? 0 : 1;
}
