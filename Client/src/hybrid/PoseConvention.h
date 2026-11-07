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
// Active body-to-world rotation R = Ry(yaw) Rx(pitch) Rz(roll).
// Presenter supplies yaw=solver[1], pitch=solver[0], roll=solver[2].
// This internal convention is self-consistent; calibration learns the IMU axes.
// Euler angles are unique for |pitch|<90. At the poles roll is chosen as zero;
// orientation still round-trips, but yaw/roll individually cannot be recovered.
Quat poseQuaternion(double yaw, double pitch, double roll);
Pose6 quaternionPose(Quat q, Vec3 translation={});
}
