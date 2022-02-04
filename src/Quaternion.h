#ifndef QUATERNION_H
#define QUATERNION_H

struct Quaternion {
    double a;
    double b;
    double c;
    double d;

    Quaternion(): a(0), b(0), c(0), d(0) {}
    Quaternion(double a, double b, double c, double d):
        a(a), b(b), c(c), d(d) {}

    Quaternion operator +(Quaternion const &r) const {
        Quaternion const &l = *this;
        return Quaternion(l.a + r.a, l.b + r.b, l.c + r.c, l.d + r.d);
    }
    Quaternion operator -() const {
        return Quaternion(-a, -b, -c, -d);
    }
    Quaternion operator -(Quaternion const &that) const {
        return *this + -that;
    }

    Quaternion operator *(double const &s) const {
        return Quaternion(s*a, s*b, s*c, s*d);
    }
    Quaternion operator /(double const &s) const {
        return *this * (1/s);
    }
    Quaternion operator *(Quaternion const &r) const {
        Quaternion const &l = *this;
        return Quaternion(
            l.a*r.a - l.b*r.b - l.c*r.c - l.d*r.d,
            l.a*r.b + l.b*r.a + l.c*r.d - l.d*r.c,
            l.a*r.c + l.c*r.a + l.d*r.b - l.b*r.d,
            l.a*r.d + l.d*r.a + l.b*r.c - l.c*r.b
        );
    }

    template<typename T>
    void operator +=(T const &that) {
        *this = *this + that;
    }

    template<typename T>
    void operator *=(T const &that) {
        *this = *this * that;
    }

    Quaternion conjugate() const {
        return Quaternion(a, -b, -c, -d);
    }

    double norm() const {
        return sqrt(a*a + b*b + c*c + d*d);
    }

    Quaternion versor() const {
        return *this / norm();
    }

    static Quaternion fromEulerAngles(double yaw, double roll, double pitch) {
        double sy = sin(yaw/2);
        double sp = sin(pitch/2);
        double sr = sin(roll/2);
        double cy = cos(yaw/2);
        double cp = cos(pitch/2);
        double cr = cos(roll/2);

        return Quaternion(
            cr*cp*cy + sr*sp*sy,
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy
        );
    }

    double yaw() const {
        return atan2(2*(a*d + b*c), 1 - 2*(c*c + d*d));
    }
    double pitch() const {
        return atan2(2*(a*b + c*d), 1 - 2*(b*b + c*c));
    }
    double roll() const {
        return asin(2*(a*c - d*b));
    }

    Quaternion lerp_norm(Quaternion const &r, double t) const {
        double lt = 1-t;
        return Quaternion(lt*a + t*r.a, lt*b + t*r.b, lt*c + t*r.c, lt*d + t*r.d).versor();
    }
};

#endif
