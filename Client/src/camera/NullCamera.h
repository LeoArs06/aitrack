#pragma once
#include "Camera.h"

class NullCamera : public Camera
{
public:
	NullCamera(int width = -1, int height = -1, int fps = 0) : Camera(width, height, fps){};
	~NullCamera() {};
	void start_camera() {};
	void stop_camera() {};
	bool get_frame(uint8_t* buffer) { return false; };
	void set_settings(CameraSettings& settings) {};
	CameraSettings get_settings() { return CameraSettings(); };
};
