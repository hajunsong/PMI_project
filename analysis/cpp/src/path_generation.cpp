#include "controlmain.h"

struct QuinticCoeffs{
    double a0, a1, a2, a3, a4, a5;
};

QuinticCoeffs quintic_coeffs(double pos0, double posf, double vel0, double velf, double acc0, double accf, double ts){
    QuinticCoeffs c;
    c.a0 = pos0;
    c.a1 = vel0;
    c.a2 = acc0/2.0;
    c.a3 = (20*(posf - pos0) - (8*velf + 12*vel0)*ts - (3*acc0 - accf)*pow(ts, 2))/(2*pow(ts, 3));
    c.a4 = (30*(pos0 - posf) + (14*velf + 16*vel0)*ts + (3*acc0 - 2*accf)*pow(ts, 2))/(2*pow(ts, 4));
    c.a5 = (12*(posf - pos0) - (6*velf + 6*vel0)*ts - (acc0 - accf)*pow(ts, 2))/(2*pow(ts, 5));
    return c;
}

std::vector< std::array<double, 3> > quintic_path(double pos0, double posf, double vel0, double velf, double acc0, double accf, double ts, double h){
    QuinticCoeffs c = quintic_coeffs(pos0, posf, vel0, velf, acc0, accf, ts);
    int n_interval = std::max(1, static_cast<int>(std::round(ts/h)));
    
    std::vector< std::array<double, 3> > path;
    double pos = 0, vel = 0, acc = 0;
    for(int i = 0; i <= n_interval; i++){
        double t = ts*static_cast<double>(i)/static_cast<double>(n_interval);
        double t2 = t*t;
        double t3 = t2*t;
        double t4 = t3*t;
        double t5 = t4*t;
        pos = c.a0 + c.a1*t + c.a2*t2 + c.a3*t3 + c.a4*t4 + c.a5*t5;
        vel = c.a1 + 2 * c.a2*t + 3*c.a3*t2 + 4*c.a4*t3 + 5*c.a5*t4;
        acc = 2*c.a2 + 6*c.a3*t + 12*c.a4*t2 + 20*c.a5*t3;
        path.push_back({pos, vel, acc});
    }
    return path;
}

std::vector< std::array<double, 3> > ControlMain::path_generation(double x0, double xf, double tf, double ta, double h, bool full_quintic){
    if(full_quintic){
        std::vector< std::array<double, 3> > path = quintic_path(x0, xf, 0.0, 0.0, 0.0, 0.0, tf, h);
        return path;
    }
    else{
        double td = tf - ta;
        double vd = (xf - x0)/td;
        double xa = x0 + 0.5*ta*vd;
        double xd = xf - 0.5*ta*vd;

        std::vector< std::array<double, 3> > pv1, pv2, pv3;

        pv1 = quintic_path(x0, xa, 0.0, vd, 0.0, 0.0, ta, h);
        pv2 = quintic_path(x0, xd, vd, vd, 0.0, 0.0, td - ta, h);
        pv3 = quintic_path(xd, xf, vd, 0.0, 0.0, 0.0, tf - td, h);

        std::vector< std::array<double, 3> > path;
        path.reserve(pv1.size() + pv2.size() + pv3.size());
        path.insert(path.end(), pv1.begin(), pv1.end());
        path.insert(path.end(), pv2.begin(), pv2.end());
        path.insert(path.end(), pv3.begin(), pv3.end());

        return path;
    }
}