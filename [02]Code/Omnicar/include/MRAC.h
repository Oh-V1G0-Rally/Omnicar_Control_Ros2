#ifndef MRAC_H
#define MRAC_H

class MRAC {
public:
    void init(double A, double tau, double dt);
    void compute(double x);
    void hammerstein(float &mmot);
    void enable(bool e);
    double Am;
    double Bm;
    double A;
    double B;
    double kx;
    double kr;
    double theta;
    double theta2;
    double xm;
    double gamma_x;
    double gamma_r;
    double gamma_theta;
    double gamma_theta2;
    double dt;
    double r;
    double u;
    bool active;
    float m, m_max;
    float hamm_vd, hamm_v0;


    
};

#endif // MRAC_H
