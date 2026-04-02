#include "MRAC.h"
#include "Robot.h"

void MRAC::init(double A, double tau, double dt) {
    this->Am = -1 / (tau / 2);
    this->Bm = 1 / (tau / 2);
    this->A = -1 / tau;
    this->B = A / tau;
    this->kx = (Am - this->A) / this->B;
    this->kr = Bm / this->B;
    this->theta = 0;
    this->theta2 = 0;
    this->xm = 0;
    this->gamma_x = 0.00001;
    this->gamma_r = 0.000005;
    this->gamma_theta = 0.0005;
    this->gamma_theta2 = 0.000000001;
    this->dt = dt;
    this->active=true;
    
}

void MRAC::compute(double x) {
    this->u = kr * r + kx * x + theta * 1 + theta2 * 0 * 0;
    
    double e = x - xm;
    kx = kx + (-gamma_x * x * e) * dt;
    kr = kr + (-gamma_r * r * e) * dt;
    theta = theta + (-gamma_theta * 1 * e);
    theta2 = theta2 + (-gamma_theta2 * 1 * e);
    xm = xm + (r * Bm + xm * Am) * dt;

    // Saturation
    if (u > m_max)
    {
      u = m_max;
    }
    else if (u < -m_max)
    {
      u = -m_max;
    }
    
}


void MRAC::hammerstein(float &mmot)
{
  if (mmot > hamm_vd)
  {
    mmot = (mmot - hamm_vd) + hamm_v0;
  }
  else if (mmot <= -hamm_vd)
  {
    mmot = (mmot + hamm_vd) - hamm_v0;
  }
  else
  {
    if (hamm_vd != 0 )
    {
      mmot = mmot * hamm_v0 / hamm_vd;
    }
    else
    {
      mmot = 0;
    }
  }
}

void MRAC::enable(bool e)
{
  if (active != e)
  {
    active = e;
  }
}
