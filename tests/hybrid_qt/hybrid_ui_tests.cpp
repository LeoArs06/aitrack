#include <QApplication>
#include <QMainWindow>
#include <QFontDatabase>
#include "ui_MainWindow.h"
#include <iostream>

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    QMainWindow window;
    Ui::MainWindow ui;
    ui.setupUi(&window);
    window.layout()->setSizeConstraint(QLayout::SetFixedSize);
    auto font = QFontDatabase::systemFont(QFontDatabase::FixedFont); font.setPointSize(8);
    ui.hybridStatus->setFont(font);
    ui.hybridMode->setCurrentIndex(1);
    ui.hybridStatus->setText("SIM 0x68 hybrid | 60ms | 0.2deg/min\nIMU 200Hz | lost 0 | CRC 0 | optical 30Hz");
    window.show(); app.processEvents();
    bool ok = window.width() >= 420 && window.width() <= 440 && window.height() < 850;
    QWidget* controls[] = {ui.hybridGroup, ui.imuControls, ui.hybridStatus};
    for (QWidget* control : controls)
        ok &= control->parentWidget()->rect().contains(control->geometry());
    window.grab().save(argc > 1 ? QString::fromLocal8Bit(argv[1]) : "hybrid-ui.png");
    ui.hybridStatus->setText(QString::fromUtf8("SIM | Move your head slowly: left-right, up-down, tilt\xe2\x80\xa6\nIMU 200 Hz | lost 0 | CRC 0 | optical 30 Hz"));
    app.processEvents();
    ok &= window.height() < 850 && ui.imuControls->rect().contains(ui.hybridStatus->geometry());
    std::cerr << "Qt main-window layout: " << (ok ? "PASS" : "FAIL") << " (" << window.width() << " x " << window.height() << ")\n";
    return ok ? 0 : 1;
}
