#pragma once
#include <QComboBox>
#include <QSerialPortInfo>
#include <QSignalBlocker>

// Refresh at popup time so plugged/unplugged devices appear without a restart.
class ImuPortComboBox : public QComboBox {
public:
    explicit ImuPortComboBox(QWidget* parent = nullptr) : QComboBox(parent) { addItem("Auto"); }
    void showPopup() override {
        const QSignalBlocker blocker(this);
        const auto selected = currentText();
        clear(); addItem("Auto");
        for (const auto& port : QSerialPortInfo::availablePorts()) addItem(port.portName());
        if (findText(selected) < 0 && !selected.isEmpty()) addItem(selected);
        setCurrentText(selected.isEmpty() ? "Auto" : selected);
        QComboBox::showPopup();
    }
};
