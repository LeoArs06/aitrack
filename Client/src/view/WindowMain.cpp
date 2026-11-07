#include "WindowMain.h"

#include "../presenter/presenter.h"
#include "../version.h"
#include <iostream>
#include <QMessageBox>
#include <QSignalBlocker>
#include <QFontDatabase>


WindowMain::WindowMain(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	this->layout()->setSizeConstraint(QLayout::SetFixedSize);

	this->setWindowTitle(QString("%1 %2 - by ARCUS LAB (based on AITrack %3)").arg(ARES_NAME, ARES_VERSION, AITRACK_VERSION));

	this->conf_win = new ConfigWindow(this);
	this->conf_win->hide();

	this->presenter = NULL;

	btn_track = findChild<QPushButton*>("trackBtn");
	tracking_frame = findChild<QLabel*>("cameraView");


	btn_config = findChild<QPushButton*>("btnConfig");

	check_video_preview = findChild<QCheckBox*>("chkVideoPreview");
	
	connect(btn_track, SIGNAL(released()), this, SLOT(onTrackClick()));
	connect(btn_config, SIGNAL(released()), this, SLOT(onConfigClick()));
	connect(check_video_preview, SIGNAL(released()), this, SLOT(onSaveClick()));
	
	statusBar()->setSizeGripEnabled(false);
	ui.hybridStatus->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
	auto statusFont = ui.hybridStatus->font(); statusFont.setPointSize(8);
	ui.hybridStatus->setFont(statusFont);
	connect(ui.hybridMode, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
		update_hybrid_controls();
		if (presenter) presenter->set_hybrid_mode(index == 1);
	});
	connect(ui.imuPort, &QComboBox::currentTextChanged, this, [this](const QString& port) {
		if (presenter) presenter->set_imu_port(port.toStdString());
	});
	connect(ui.imuConnect, &QPushButton::clicked, this, [this](bool requested) {
		ui.imuConnect->setText(requested ? "Disconnect" : "Connect");
		if (presenter) presenter->connect_imu(requested);
	});
	connect(ui.imuCalibrate, &QPushButton::clicked, this, [this] {
		if (presenter) presenter->calibrate_imu(!imu_calibrating_);
	});
	connect(ui.opticalStrength, &QSlider::valueChanged, this, [this](int strength) {
		ui.opticalValue->setText(QString::number(strength));
		if (presenter) presenter->set_optical_strength(strength);
	});
	update_hybrid_controls();
}

WindowMain::~WindowMain()
{}

void WindowMain::closeEvent(QCloseEvent* event)
{
	// note: deleting nullptr has no effect
	delete(this->toggle_tracking_shortcut);
	this->presenter->close_program();
}


void WindowMain::paint_video_frame(cv::Mat& img)
{
	if (check_video_preview->isChecked())
		tracking_frame->setPixmap(
			QPixmap::fromImage(QImage(img.data, img.cols, img.rows, img.step, QImage::Format_RGB888))
				.scaled(400,280, Qt::KeepAspectRatio, Qt::FastTransformation)
		);
}


void WindowMain::show_tracking_data(ConfigData conf)
{
	tracking_info->setText(QString("X: %1, Y: %2, Z: %3")
		.arg((int)conf.x)
		.arg((int)conf.y)
		.arg((int)conf.z));
}


void WindowMain::connect_presenter(IPresenter* presenter)
{
	if (presenter != NULL && this->presenter == nullptr)
	{
		this->presenter = presenter;
		conf_win->connect_presenter(presenter);
	}
}

void WindowMain::set_tracking_mode(bool is_tracking)
{
	tracking_ = is_tracking;
	update_hybrid_controls();
	if (is_tracking)
	{
		// Change button name to "stop"
		btn_track->setText("Stop tracking");
	}
	else
	{
		// Change button name to "start"
		btn_track->setText("Start tracking");

		// Remove background from label
		tracking_frame->setPixmap(QPixmap());
		tracking_frame->setText("Tracking stopped.");
	}

	btn_config->setDisabled(is_tracking);
	conf_win->set_tracking_mode(is_tracking);
}


void WindowMain::update_view_state(ConfigData conf)
{
	set_inputs(conf);
	//show_tracking_data(conf);

	if (!conf.show_video_feed)
	{
		tracking_frame->hide();
		readjust_size();
	}
	else
	{
		tracking_frame->show();
	}
}

ConfigData WindowMain::get_inputs()
{
	// Obtain inputs of children windows
	ConfigData inputs = conf_win->get_inputs();
	inputs.show_video_feed = check_video_preview->isChecked();
	return inputs;
}

void WindowMain::set_inputs(const ConfigData data)
{
	const QSignalBlocker modeBlock(ui.hybridMode), portBlock(ui.imuPort), strengthBlock(ui.opticalStrength);
	ui.hybridMode->setCurrentIndex(data.hybrid_mode ? 1 : 0);
	const auto port = QString::fromStdString(data.imu_port);
	if (ui.imuPort->findText(port) < 0) ui.imuPort->addItem(port);
	ui.imuPort->setCurrentText(port);
	ui.opticalStrength->setValue(data.optical_strength);
	ui.opticalValue->setText(QString::number(data.optical_strength));
	update_hybrid_controls();
	check_video_preview->setChecked(data.show_video_feed);
	this->conf_win->update_view_state(data);
}

void WindowMain::show_message(const char* msg, MSG_SEVERITY severity)
{
	QMessageBox msgBox;
	switch (severity) {
	case CRITICAL:
		msgBox.setIcon(QMessageBox::Warning);
		break;
	default:
		msgBox.setIcon(QMessageBox::Information);
		break;
	}

	msgBox.setText(msg);
	msgBox.exec();
}

void WindowMain::set_enabled(bool enabled)
{
	btn_track->setEnabled(enabled);

	conf_win->set_enabled(enabled);
}

void WindowMain::onTrackClick()
{
	std::cout << "TOGGLE TRACKING" << std::endl;
	presenter->toggle_tracking();
}

void WindowMain::onSaveClick()
{
	// Merge with config from this window
	ConfigData config = get_inputs();
	
	presenter->save_prefs(config);

	std::cout << "Saved changes" << std::endl;
}

void WindowMain::onConfigClick()
{
	this->conf_win->show();
}

void WindowMain::readjust_size()
{
	findChild<QWidget*>("centralwidget")->adjustSize();
	adjustSize();
}


void WindowMain::notify(IView* self)
{
	this->onSaveClick();
}


void WindowMain::set_shortcuts(bool enabled)
{
	// referer to program state instead to avoid QT routines
	if (enabled) {
		if (this->toggle_tracking_shortcut == nullptr) {
			std::cout << "Enabling tracking shortcut" << std::endl;
			this->toggle_tracking_shortcut = new QGlobalShortcut();
			this->toggle_tracking_shortcut->setKey(QKeySequence("Ctrl+T"));
			connect(this->toggle_tracking_shortcut, SIGNAL(activated()), SLOT(onTrackClick()));
		}
		else {
			std::cerr << "Tracking shortcut has already been enabled!" << std::endl;
		}
	}
	else {
		std::cout << "Disabling tracking shortcut" << std::endl;
		// note: deleting nullptr has no effect
		delete this->toggle_tracking_shortcut;
		this->toggle_tracking_shortcut = nullptr;
	}
}

IView* WindowMain::get_calibration_window()
{
	return conf_win->get_calibration_window();
}

void WindowMain::update_hybrid_controls() {
	const bool hybrid = ui.hybridMode->currentIndex() == 1;
	ui.imuControls->setEnabled(hybrid);
	ui.imuCalibrate->setEnabled(hybrid && imu_connected_ && tracking_);
	ui.imuCalibrate->setText(imu_calibrating_ ? "Cancel" : "Calibrate");
}

void WindowMain::set_hybrid_status(const QString& text, const QString& progress,
	bool connected, bool requested, bool calibrating) {
	imu_connected_ = connected; imu_calibrating_ = calibrating;
	const QSignalBlocker blocker(ui.imuConnect);
	ui.imuConnect->setChecked(requested);
	ui.imuConnect->setText(requested ? "Disconnect" : "Connect");
	const auto prefix = text.startsWith("SIM") ? QString("SIM | ") : QString();
	ui.hybridStatus->setText(progress.isEmpty() ? text : prefix + progress + '\n' + text.section('\n', 1));
	ui.hybridStatus->setToolTip(progress + '\n' + text);
	update_hybrid_controls();
}
