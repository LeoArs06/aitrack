#pragma once
#include <algorithm>
#include <cmath>

namespace hybrid {
constexpr double pi = 3.14159265358979323846;
constexpr double radians = pi / 180.0;
constexpr double degrees = 180.0 / pi;
struct Vec3 {
    double x=0, y=0, z=0;
    double& operator[](int i) { return i==0 ? x : (i==1 ? y : z); }
    double operator[](int i) const { return i==0 ? x : (i==1 ? y : z); }
    Vec3 operator+(Vec3 b) const { return {x+b.x,y+b.y,z+b.z}; }
    Vec3 operator-(Vec3 b) const { return {x-b.x,y-b.y,z-b.z}; }
    Vec3 operator-() const { return {-x,-y,-z}; }
    Vec3 operator*(double s) const { return {x*s,y*s,z*s}; }
    Vec3 operator/(double s) const { return *this*(1/s); }
    Vec3& operator+=(Vec3 b) { *this=*this+b; return *this; }
    double dot(Vec3 b) const { return x*b.x+y*b.y+z*b.z; }
    Vec3 cross(Vec3 b) const { return {y*b.z-z*b.y,z*b.x-x*b.z,x*b.y-y*b.x}; }
    double norm() const { return std::sqrt(dot(*this)); }
    bool finite() const { return std::isfinite(x)&&std::isfinite(y)&&std::isfinite(z); }
};
inline Vec3 operator*(double s, Vec3 v) { return v*s; }
struct Mat3 {
    double m[3][3]={{1,0,0},{0,1,0},{0,0,1}};
    Vec3 operator*(Vec3 v) const {
        return {m[0][0]*v.x+m[0][1]*v.y+m[0][2]*v.z,
                m[1][0]*v.x+m[1][1]*v.y+m[1][2]*v.z,
                m[2][0]*v.x+m[2][1]*v.y+m[2][2]*v.z};
    }
};
struct Quat {
    double w=1,x=0,y=0,z=0;
    Quat operator*(Quat b) const {
        return {w*b.w-x*b.x-y*b.y-z*b.z, w*b.x+x*b.w+y*b.z-z*b.y,
                w*b.y-x*b.z+y*b.w+z*b.x, w*b.z+x*b.y-y*b.x+z*b.w};
    }
    Quat conj() const { return {w,-x,-y,-z}; }
    double dot(Quat b) const { return w*b.w+x*b.x+y*b.y+z*b.z; }
    bool finite() const { return std::isfinite(w)&&std::isfinite(x)&&std::isfinite(y)&&std::isfinite(z); }
    Quat normalized() const {
        double n=std::sqrt(dot(*this));
        return n>1e-15 && std::isfinite(n) ? Quat{w/n,x/n,y/n,z/n} : Quat{};
    }
    Vec3 rotate(Vec3 v) const { Vec3 u{x,y,z}; return v+2*w*u.cross(v)+2*u.cross(u.cross(v)); }
    // SO(3) exponential/logarithm: vector length is the full rotation in radians.
    static Quat exp(Vec3 v) {
        double a=v.norm(), k=a<1e-8 ? 0.5-a*a/48 : std::sin(a/2)/a;
        return Quat{std::cos(a/2),v.x*k,v.y*k,v.z*k}.normalized();
    }
    Vec3 log() const {
        Quat q=normalized();
        if(q.w<0) q={-q.w,-q.x,-q.y,-q.z};
        double s=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z);
        double k=s<1e-10 ? 2 : 2*std::atan2(s,q.w)/s;
        return {q.x*k,q.y*k,q.z*k};
    }
    Mat3 matrix() const {
        Quat q=normalized(); double a=q.w,b=q.x,c=q.y,d=q.z;
        return {{{1-2*(c*c+d*d),2*(b*c-a*d),2*(b*d+a*c)},
                 {2*(b*c+a*d),1-2*(b*b+d*d),2*(c*d-a*b)},
                 {2*(b*d-a*c),2*(c*d+a*b),1-2*(b*b+c*c)}}};
    }
    static Quat fromMatrix(const Mat3& r) {
        const auto& m=r.m; Quat q; double t=m[0][0]+m[1][1]+m[2][2];
        if(t>0) { double s=2*std::sqrt(1+t); q={s/4,(m[2][1]-m[1][2])/s,(m[0][2]-m[2][0])/s,(m[1][0]-m[0][1])/s}; }
        else {
            int i=0; if(m[1][1]>m[0][0]) i=1; if(m[2][2]>m[i][i]) i=2;
            int j=(i+1)%3,k=(i+2)%3; double s=2*std::sqrt(std::max(0.0,1+m[i][i]-m[j][j]-m[k][k]));
            double v[3]={0,0,0}; v[i]=s/4; v[j]=(m[j][i]+m[i][j])/s; v[k]=(m[k][i]+m[i][k])/s;
            q={(m[k][j]-m[j][k])/s,v[0],v[1],v[2]};
        }
        return q.normalized();
    }
    static Quat slerp(Quat a, Quat b, double t) { return (a*exp((a.conj()*b).log()*std::clamp(t,0.0,1.0))).normalized(); }
};
inline double angularDistance(Quat a, Quat b) { return (a.conj()*b).log().norm(); }
}
