#pragma once
#include "Quat.h"
namespace hybrid {
struct Pose6 {
    double x=0,y=0,z=0,yaw=0,pitch=0,roll=0;
    // Exact Presenter::send_data ordering, centimetres and degrees.
    void packet(double (&out)[6]) const;
};
struct OpticalPose {
    bool valid=false;
    double yaw=0,pitch=0,roll=0,x=0,y=0,z=0;
};
// AITrack's own convention, verified against cv::decomposeProjectionMatrix
// (OpenCV 4.6, max |diff| 1e-13): R = Rz(roll) Ry(yaw) Rx(pitch), with
// pitch=solver[0], yaw=solver[1], roll=solver[2] (Presenter mapping).
// Calibration learns the IMU->head axes; this mapping must match AITrack
// exactly, otherwise gyro and optical disagree at combined angles.
// Unique for |yaw|<90 (AITrack clips yaw to +-90). At the poles pitch=0.
Quat poseQuaternion(double yaw, double pitch, double roll);
Pose6 quaternionPose(Quat q, Vec3 translation={});
}
