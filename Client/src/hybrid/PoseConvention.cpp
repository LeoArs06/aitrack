#include "PoseConvention.h"
namespace hybrid {
void Pose6::packet(double (&out)[6]) const { out[0]=x; out[1]=y; out[2]=z; out[3]=yaw; out[4]=pitch; out[5]=roll; }
Quat poseQuaternion(double yaw,double pitch,double roll) {
    return (Quat::exp({0,yaw*radians,0})*Quat::exp({pitch*radians,0,0})*Quat::exp({0,0,roll*radians})).normalized();
}
Pose6 quaternionPose(Quat q,Vec3 t) {
    auto r=q.matrix(); double p=std::asin(std::clamp(-r.m[1][2],-1.0,1.0));
    double y, z;
    if(std::abs(std::cos(p))>1e-7) { y=std::atan2(r.m[0][2],r.m[2][2]); z=std::atan2(r.m[1][0],r.m[1][1]); }
    else { y=std::atan2(-r.m[2][0],r.m[0][0]); z=0; }
    return {t.x,t.y,t.z,y*degrees,p*degrees,z*degrees};
}
}
