#pragma once

#include "../model/Config.h"
//#include "../view/i_view.h"

class IView;

class IPresenter
{
public:
	virtual void set_hybrid_mode(bool hybrid) = 0;
	virtual void set_imu_port(const std::string& port) = 0;
	virtual void connect_imu(bool connect) = 0;
	virtual void calibrate_imu(bool start) = 0;
	virtual void set_optical_strength(int strength) = 0;
	virtual void toggle_tracking() = 0;
	virtual void save_prefs(const ConfigData& data) = 0;
	virtual void calibrate_face(IView& view) = 0;
	virtual void close_program() = 0;
};
