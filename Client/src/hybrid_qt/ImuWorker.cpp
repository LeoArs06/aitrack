// Winsock must precede Qt's Windows headers.
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include "../model/UDPSender.h"
#include "ImuWorker.h"
#include "HostClock.h"
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTimer>
#include <QDateTime>
#include <QDir>
#include <QLocale>
#include <algorithm>

namespace hybrid_qt {
ImuWorker::ImuWorker(QObject* parent) : QObject(parent) {}
ImuWorker::~ImuWorker() = default;

void ImuWorker::initialize() {
    sim_ = qEnvironmentVariable("AITRACK_IMU_SIM") == "1";
    serial_ = new QSerialPort(this);
    connect(serial_, &QSerialPort::readyRead, this, [this] {
        const auto arrival = hostNowUs();
        const auto bytes = serial_->readAll();
        receive(reinterpret_cast<const uint8_t*>(bytes.constData()), size_t(bytes.size()), arrival);
    });
    // Queue errors to avoid closing a device from inside QSerialPort::open/read.
    connect(serial_, &QSerialPort::errorOccurred, this, [this](QSerialPort::SerialPortError error) {
        if (stopping_ || !serial_->isOpen() || (!probing_ && !connected_)) return;
        if (error == QSerialPort::ResourceError || error == QSerialPort::DeviceNotFoundError ||
            error == QSerialPort::PermissionError || error == QSerialPort::ReadError) {
            const auto port = serial_->portName();
            const auto reason = "Serial error: " + serial_->errorString();
            QMetaObject::invokeMethod(this, [this, port, reason] {
                if (serial_->portName() != port || !serial_->isOpen()) return;
                const bool wasProbing = probing_;
                closePort(reason);
                if (wasProbing && requested_) nextCandidate();
                report();
            }, Qt::QueuedConnection);
        }
    });
    tick_ = new QTimer(this);
    tick_->setTimerType(Qt::PreciseTimer);
    tick_->setInterval(5);
    connect(tick_, &QTimer::timeout, this, &ImuWorker::tick);
    tick_->start();
    diagnosticsTimer_ = new QTimer(this);
    diagnosticsTimer_->setInterval(250);
    connect(diagnosticsTimer_, &QTimer::timeout, this, &ImuWorker::report);
    diagnosticsTimer_->start();
    lastRateReport_ = hostNowUs();
}

void ImuWorker::configure(QString ip, int port, int strength, QString calibration, bool logging) {
    ip = ip.trimmed().remove(' ');
    if (ip.size() < 2) ip = "127.0.0.1";
    if (port == 0) port = 4242;
    if (!sender_ || sender_->ip != ip.toStdString() || sender_->port != port)
        sender_ = std::make_unique<UDPSender>(ip.toStdString().c_str(), port);
    strength_ = std::clamp(strength, 0, 100);
    fusion_.setStrength(strength_ / 100.0);
    hybrid::CalibrationResult c;
    if (calibration != appliedCalibration_ && !calibration.isEmpty() &&
        hybrid::CalibrationResult::deserialize(calibration.toStdString(), c)) {
        appliedCalibration_ = calibration;
        savedCalibration_ = c; calibrated_ = true;
        fusion_.setCalibration(c);
    }
    openLog(logging);
}

void ImuWorker::resetFusion() {
    fusion_ = hybrid::HybridFusion{};
    fusion_.setStrength(strength_ / 100.0);
    fusion_.setCalibration(savedCalibration_);
}

void ImuWorker::setPort(QString port) {
    if (selectedPort_ == port) return;
    selectedPort_ = port;
    const bool reconnect = requested_;
    requestConnection(false);
    if (reconnect) requestConnection(true);
}

void ImuWorker::requestConnection(bool requested) {
    if (stopping_) return;
    if (requested_ == requested && (connected_ || probing_)) return;
    requested_ = requested;
    if (!requested) {
        calibrate(false);
        closePort("Disconnected");
        simulator_.reset();
    } else if (sim_) {
        parser_.reset(); clock_.reset(); firstSample_ = true;
        hybrid::Simulator::Config c;
        c.durationSec = 30; c.hostOffsetUs = 0; c.mcuOffsetUs = 0;
        c.motion = hybrid::Simulator::Motion::Calibration;
        simulator_ = std::make_unique<hybrid::Simulator>(c);
        simStart_ = hostNowUs(); simIndex_ = 0; simCycle_ = 0;
        probing_ = true;
        const auto bytes = hybrid::encode(hybrid::Info{});
        receive(bytes.data(), bytes.size(), simStart_);
    } else {
        beginScan();
    }
    report();
}

void ImuWorker::beginScan() {
    candidates_.clear();
    auto ports = QSerialPortInfo::availablePorts();
    std::stable_sort(ports.begin(), ports.end(), [this](const QSerialPortInfo& a, const QSerialPortInfo& b) {
        auto rank = [this](const QSerialPortInfo& p) {
            if (p.portName() == lastGoodPort_) return 0;
            const auto vid = p.vendorIdentifier();
            return p.hasVendorIdentifier() && (vid == 0x2341 || vid == 0x1B4F || vid == 0x2A03) ? 1 : 2;
        };
        return rank(a) < rank(b);
    });
    if (selectedPort_ != "Auto") candidates_ << selectedPort_;
    else for (const auto& p : ports) candidates_ << p.portName();
    nextCandidate();
}

void ImuWorker::nextCandidate() {
    if (!requested_ || stopping_) return;
    if (serial_->isOpen()) serial_->close();
    connected_ = false; probing_ = false;
    while (!candidates_.isEmpty()) {
        const auto port = candidates_.takeFirst();
        serial_->setPortName(port);
        serial_->setBaudRate(115200); // Never use the 1200-baud bootloader trigger.
        serial_->setDataBits(QSerialPort::Data8);
        serial_->setParity(QSerialPort::NoParity);
        serial_->setStopBits(QSerialPort::OneStop);
        serial_->setFlowControl(QSerialPort::NoFlowControl);
        if (!serial_->open(QIODevice::ReadWrite)) continue;
        parser_.reset(); clock_.reset(); firstSample_ = true;
        probing_ = true; probeStart_ = hostNowUs();
        connection_ = "Probing " + port;
        if (!serial_->setDataTerminalReady(true)) { serial_->close(); probing_ = false; continue; }
        serial_->write("?\n");
        return;
    }
    connection_ = "IMU unavailable; retrying";
    retryAt_ = hostNowUs() + 1000000;
}

void ImuWorker::closePort(QString reason) {
    const bool hadConnection = connected_;
    connected_ = false; probing_ = false;
    if (serial_ && serial_->isOpen()) serial_->close();
    connection_ = std::move(reason);
    retryAt_ = hostNowUs() + 1000000;
    // Retain fusion history during outages so its optical crossfade can run.
    if (hadConnection && calibrating_) {
        calibrating_ = false;
        progress_ = "IMU disconnected";
    }
}

void ImuWorker::receive(const uint8_t* bytes, size_t size, int64_t arrival) {
    for (const auto& message : parser_.feed(bytes, size)) {
        if (const auto* info = std::get_if<hybrid::Info>(&message)) {
            if (info->gyroFS != 2 || info->accelFS != 1 || info->sampleHz != 200) {
                closePort("Unsupported IMU ranges/rate");
                if (requested_ && !sim_) nextCandidate();
                return;
            }
            info_ = *info;
            if (!requested_) continue;
            const bool newlyConnected = !connected_;
            connected_ = true; probing_ = false;
            if (!sim_) lastGoodPort_ = serial_->portName();
            connection_ = sim_ ? "SIM connected" : lastGoodPort_ + " connected";
            if (newlyConnected) { lastSample_ = arrival; report(); }
        } else if (const auto* raw = std::get_if<hybrid::SampleRaw>(&message)) {
            if (!connected_) continue; // INFO must validate identity and scaling first.
            auto sample = hybrid::scale(*raw, info_.whoAmI);
            const bool first = firstSample_;
            sample.discontinuity = sample.discontinuity || firstSample_;
            firstSample_ = false;
            const auto mapped = clock_.observe(raw->timeUs, arrival, sample.discontinuity);
            lastSample_ = arrival; ++imuCount_;
            if (csv_) *csv_ << "imu," << arrival << ',' << raw->seq << ',' << raw->t_us << ','
                << raw->ax << ',' << raw->ay << ',' << raw->az << ',' << raw->temp_raw << ','
                << raw->gx << ',' << raw->gy << ',' << raw->gz << ',' << int(raw->flags) << ",,,,,,,\n";
            fusion_.addImuSample(mapped, sample);
            // The first sample is a clock epoch reset, not calibration evidence.
            if (!first) auto_.addImuSample(mapped, sample);
            output(arrival);
        } else if (const auto* status = std::get_if<hybrid::Status>(&message)) {
            if (connected_ && status->code != 3) {
                closePort(QString("IMU error %1; retrying").arg(status->code));
                report();
                return;
            }
        }
    }
}

void ImuWorker::setOutputEnabled(bool enabled, quint64 generation) {
    if (enabled && !outputEnabled_) resetFusion();
    outputEnabled_ = enabled;
    if (!enabled) calibrate(false);
    emit outputAcknowledged(enabled, generation);
}

void ImuWorker::addOptical(int64_t arrival, hybrid::OpticalPose pose) {
    lastOptical_ = arrival; ++opticalCount_;
    if (csv_) *csv_ << "optical," << arrival << ",,,,,,,,,,," << int(pose.valid) << ','
        << pose.yaw << ',' << pose.pitch << ',' << pose.roll << ',' << pose.x << ',' << pose.y << ',' << pose.z << '\n';
    fusion_.addOptical(arrival, pose);
    auto_.addOptical(arrival, pose);
    // Until the mounting is learned, behave exactly like AITrack: camera pose out.
    if (connected_ && !calibrated_ && outputEnabled_ && pose.valid && sender_ && sender_->valid) {
        hybrid::Pose6 p; p.x = pose.x; p.y = pose.y; p.z = pose.z;
        p.yaw = pose.yaw; p.pitch = pose.pitch; p.roll = pose.roll;
        double packet[6]; p.packet(packet); sender_->send_data(packet);
    }
}

void ImuWorker::output(int64_t now) {
    // Advance even without a sender so diagnostics and fallback state remain current.
    const auto result = fusion_.output(now);
    if (!outputEnabled_ || !result.valid || !sender_ || !sender_->valid) return;
    if (connected_ && !calibrated_) return; // camera pass-through in addOptical()
    double packet[6]; result.pose.packet(packet);
    static_assert(sizeof(packet) == 48, "OpenTrack expects six 64-bit doubles");
    sender_->send_data(packet);
}

void ImuWorker::tick() {
    if (stopping_) return;
    const auto now = hostNowUs();
    if (sim_ && requested_ && simulator_) {
        const auto& events = simulator_->imu();
        constexpr int64_t cycleUs = 30005000; // include the sample at 30 s once
        // Bound catch-up after a suspended process; normal ticks deliver one sample.
        int budget = 20;
        while (budget-- && simCycle_ * cycleUs + events[simIndex_].arrivalUs <= uint64_t(now - simStart_)) {
            auto raw = events[simIndex_].raw();
            raw.seq += uint32_t(simCycle_ * events.size());
            raw.t_us += uint32_t(simCycle_ * cycleUs);
            if (simCycle_ && simIndex_ == 0) raw.flags |= 4;
            const auto bytes = hybrid::encode(raw);
            receive(bytes.data(), bytes.size(), hostNowUs());
            if (++simIndex_ == events.size()) { simIndex_ = 0; ++simCycle_; }
        }
    } else if (requested_) {
        if (probing_ && now - probeStart_ > 700000) nextCandidate();
        else if (connected_ && now - lastSample_ > 500000) {
            closePort("IMU silent; retrying"); report();
        } else if (!connected_ && !probing_ && now >= retryAt_) beginScan();
    }
    if (now - lastSample_ > 20000) output(now);
    updateCalibration(now);
}

// Calibration runs continuously in the background from ordinary head motion.
// The button only forgets the learned mounting (e.g. after moving the sensor).
void ImuWorker::calibrate(bool start) {
    if (start) {
        auto_.reset(); calibrated_ = false;
        savedCalibration_ = hybrid::CalibrationResult{}; appliedCalibration_.clear();
        resetFusion();
        emit calibrationSaved(QString());
        progress_ = "Recalibrating: use the tracker normally";
    }
    report();
}

void ImuWorker::updateCalibration(int64_t now) {
    if (!connected_ || !outputEnabled_) return;
    if (auto_.update(now)) {
        const bool first = !calibrated_;
        savedCalibration_ = auto_.result(); calibrated_ = true;
        appliedCalibration_ = QString::fromStdString(savedCalibration_.serialize());
        // First lock: start fused output fresh from the camera pose. Later
        // refinements only update parameters, keeping the output continuous.
        if (first) resetFusion(); else fusion_.setCalibration(savedCalibration_);
        emit calibrationSaved(appliedCalibration_);
        progress_ = QString("Calibrated automatically (fit %1%, camera delay %2 ms)")
            .arg(int(savedCalibration_.quality * 100)).arg(savedCalibration_.latencyMs, 0, 'f', 0);
        report();
    } else if (!calibrated_) {
        const auto p = QString("Learning sensor mounting: %1% - look around normally").arg(int(auto_.progress() * 100));
        if (p != progress_) { progress_ = p; report(); }
    }
}

void ImuWorker::report() {
    const auto now = hostNowUs();
    const double seconds = (now - lastRateReport_) * 1e-6;
    if (seconds >= 0.20) {
        imuHz_ = imuCount_ / seconds; opticalHz_ = opticalCount_ / seconds;
        imuCount_ = opticalCount_ = 0; lastRateReport_ = now;
    }
    const auto& d = fusion_.diagnostics();
    const auto& c = parser_.counters();
    QString chip = QString("0x%1").arg(info_.whoAmI, 2, 16, QLatin1Char('0'));
    if (info_.whoAmI != 0x68 && info_.whoAmI != 0x70 && info_.whoAmI != 0x71 && info_.whoAmI != 0x73)
        chip += " (unverified temp)";
    const auto source = connected_ ? (sim_ ? QString("SIM") : lastGoodPort_) : connection_;
    QString text = QString("%1 %2 %3 | %8ms | %9deg/min\nIMU %4Hz | lost %5 | CRC %6 | optical %7Hz")
        .arg(source, connected_ ? chip : "--", hybrid::stateName(d.state))
        .arg(now - lastSample_ > 500000 ? 0.0 : imuHz_, 0, 'f', 0)
        .arg(qulonglong(c.lost)).arg(qulonglong(c.crcErrors))
        .arg(now - lastOptical_ > 500000 ? 0.0 : opticalHz_, 0, 'f', 0)
        .arg(d.latencyMs, 0, 'f', 0).arg(d.driftDegPerMin, 0, 'f', 1);
    emit diagnostics(text, progress_, connected_, requested_, false);
    if (csv_) csv_->flush();
}

void ImuWorker::openLog(bool enabled) {
    if (enabled == bool(csv_)) return;
    csv_.reset(); log_.close();
    if (!enabled) return;
    QDir().mkpath("logs");
    log_.setFileName("logs/hybrid_" + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz") + ".csv");
    if (!log_.open(QIODevice::WriteOnly | QIODevice::Text)) {
        progress_ = "Cannot open hybrid CSV log: " + log_.errorString(); report(); return;
    }
    csv_ = std::make_unique<QTextStream>(&log_);
    csv_->setLocale(QLocale::c()); csv_->setRealNumberPrecision(17);
    *csv_ << "kind,host_us,seq,t_us,ax,ay,az,temp_raw,gx,gy,gz,flags,valid,yaw,pitch,roll,x,y,z\n";
}

void ImuWorker::shutdown() {
    stopping_ = true; outputEnabled_ = false; requested_ = false;
    tick_->stop(); diagnosticsTimer_->stop();
    closePort("Disconnected");
    csv_.reset(); log_.close(); simulator_.reset(); sender_.reset();
    emit stopped();
}
}
