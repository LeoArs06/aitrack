#pragma once

#include "../hybrid/HybridFusion.h"
#include "../hybrid/ClockSync.h"
#include "../hybrid/Simulator.h"
#include <QObject>
#include <QFile>
#include <QStringList>
#include <QTextStream>
#include <memory>

class QSerialPort;
class QTimer;
class UDPSender;

namespace hybrid_qt {
// Every method (including optical submissions) runs on the worker's QThread.
// Presenter only submits value-capturing queued functors; no shared core state.
class ImuWorker : public QObject {
    Q_OBJECT
public:
    explicit ImuWorker(QObject* parent = nullptr);
    ~ImuWorker() override;
    void initialize();
    void configure(QString ip, int port, int strength, QString calibration, bool logging);
    void setPort(QString port);
    void requestConnection(bool requested);
    void setOutputEnabled(bool enabled, quint64 generation);
    void addOptical(int64_t arrivalUs, hybrid::OpticalPose pose);
    void calibrate(bool start);
    void shutdown();

signals:
    void diagnostics(QString text, QString progress, bool connected, bool requested, bool calibrating);
    void calibrationSaved(QString calibration);
    void outputAcknowledged(bool enabled, quint64 generation);
    void stopped();

private:
    QSerialPort* serial_ = nullptr;
    QTimer* tick_ = nullptr;
    QTimer* diagnosticsTimer_ = nullptr;
    hybrid::ImuProtocol parser_;
    hybrid::ClockSync clock_;
    hybrid::HybridFusion fusion_;
    hybrid::AutoCalibrator auto_;
    bool calibrated_ = false; // false: send the plain camera pose (AITrack behaviour)
    hybrid::CalibrationResult savedCalibration_;
    hybrid::Info info_;
    std::unique_ptr<UDPSender> sender_;
    std::unique_ptr<hybrid::Simulator> simulator_;
    QFile log_;
    std::unique_ptr<QTextStream> csv_;
    QString selectedPort_ = "Auto", lastGoodPort_, connection_ = "Disconnected";
    QStringList candidates_;
    QString progress_;
    QString appliedCalibration_;
    bool requested_ = false, connected_ = false, probing_ = false;
    bool outputEnabled_ = false, calibrating_ = false, sim_ = false, stopping_ = false;
    bool firstSample_ = true;
    int strength_ = 50;
    int64_t probeStart_ = 0, lastSample_ = 0, retryAt_ = 0;
    int64_t simStart_ = 0, lastOptical_ = 0, lastRateReport_ = 0;
    size_t simIndex_ = 0;
    uint64_t simCycle_ = 0, imuCount_ = 0, opticalCount_ = 0;
    double imuHz_ = 0, opticalHz_ = 0;
    void tick();
    void beginScan();
    void nextCandidate();
    void closePort(QString reason);
    void receive(const uint8_t* bytes, size_t size, int64_t arrivalUs);
    void output(int64_t now);
    void report();
    void updateCalibration(int64_t now);
    void resetFusion();
    void openLog(bool enabled);
};
}
