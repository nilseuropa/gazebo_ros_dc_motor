#include "gazebo_ros_motors/bldcmotor_foc.h"

double saturate(double value, const double& absmax);

nowtech::Vec4 BLDCMotorFoC::f(const nowtech::Vec4& y0, const double& t) const {
  (void)t; // just to suppress warning to unused parameter during build

  /// State variables from y0
  double id = y0(0);
  double iq = y0(1);
  double omega = y0(2); // mechanical angular vel.
//  double theta = y0(3); // mechanical angle
  // In the equations: omega = mechanical angle/ang-vel, electrical angle/angular veloc is p*omega
  double iddot = (Vd - R * id + (p * omega) * iq * Lq) / Ld;
  double iqdot = (Vq - R * iq - (p * omega) * (id * Ld + lpm)) / Lq;
  double TE = (3.0 / 2.0) * p * (lpm * iq + (Ld - Lq) * id * iq);
  double omegadot = (TE - Tload - beta * omega) / J;
  double thetadot = omega;
  return nowtech::Vec4({iddot, iqdot, omegadot, thetadot});
}

void BLDCMotorFoC::speed_control_pi(const nowtech::Vec4& state) {
  double omega_m = state(2); // mechanical angular vel.
  double prop_err = omega_t - omega_m;
  v_int_err += dt * prop_err;
  Tref = kp_v * prop_err + ki_v * v_int_err;
}

void BLDCMotorFoC::foc_control_surface_pmsm(const nowtech::Vec4& state) {
  double id = state(0);
  double iq = state(1);
  double omega_m = state(2); // mechanical angular vel.
  // double theta_m = state(3); // mechanical angle
  double omega_e = p * omega_m;
  // double theta_e = p * theta_m;
  // Calculate base speed which depends on actual d-q currents
  double omega_base = Uref / (p * sqrt((Lq * iq) * (Lq * iq) + (Ld * id + lpm) * (Ld * id + lpm)));
  if (omega_m <= omega_base) {
    double iq_mtpa = 2.0 * Tref / (3.0 * p * lpm);
    id_sat = 0.0;
    iq_sat = saturate(iq_mtpa, imax);
  } else {
    /* omega_m > omega_base, field weakening operation */
    double id_fw = (omega_base - omega_m) * lpm / (omega_m * Ld);
    id_sat = saturate(id_fw, imax);
    double iq_fw = 2.0 * Tref / (3.0 * p * lpm);
    double iq_limit = sqrt(imax * imax - id_sat * id_sat);
    iq_sat = saturate(iq_fw, iq_limit);
  }
  // Current PI control
  double id_prop_err = id_sat - id;
  double iq_prop_err = iq_sat - iq;
  id_int_err += dt * id_prop_err;
  iq_int_err += dt * iq_prop_err;
  double Vd_FF = R*id-omega_e*Lq*iq; // Feed-forward based on equilibrium (rate of change of id & iq is zero)
  double Vq_FF = R*iq+omega_e*(Ld*id+lpm);
  Vd = (kp_c*id_prop_err + ki_c*id_int_err)+Vd_FF;
  Vq = (kp_c*iq_prop_err + ki_c*iq_int_err)+Vq_FF;
}

nowtech::Vec4 BLDCMotorFoC::step(const nowtech::Vec4& state) {
  // Execute speed-controller to set required torque
  speed_control_pi(state);
  // Execute MTPA current controller to set appropriate voltages
  foc_control_surface_pmsm(state);
  // Use RK45 to integrate the system for a total of dt time
  nowmath::RK45<nowtech::Vec4, BLDCMotorFoC> rk45(this, 1e-6);
  return rk45.step(state, 0, dt, dt / 10.0);
}

BLDCMotorFoC::BLDCMotorFoC(double r,
                           double kT,
                           double imax,
                           double ls,
                           double ms,
                           double lm,
                           double p,
                           double j,
                           double beta,
                           double Umax,
                           double dt) :
    R(r),
    kt(kT),
    Ls(ls),
    Ms(ms),
    Lm(lm),
    p(p),
    J(j),
    beta(beta),
    Uref(Umax),
    dt(dt) {
  Tload = 0;
  Vd = 0;
  Vq = 0;
  Ld = Ls + Ms + (3.0 / 2.0) * Lm;
  Lq = Ls + Ms - (3.0 / 2.0) * Lm;
  kp_v = ki_v = 0.0;
  kp_c = ki_c = 0.0;
  omega_t = 0.0;
  lpm = kt / p; // perm. magnet flux linkage: kt = p*lpm
  this->imax = imax; // Uref / R results too large value compared to the datasheet
  v_int_err = 0.0;
  id_int_err = iq_int_err = 0.0;
  Tref = 0;
  iq_sat = 0;
  id_sat = 0;
}

void BLDCMotorFoC::reset_controller() {
  // Reset integral errors and desired angular velocity
  v_int_err = 0.0;
  id_int_err = iq_int_err = 0.0;
  omega_t = 0;
}

void BLDCMotorFoC::set_veloc_pi(const double& kp, const double& ki) {
  kp_v = kp;
  ki_v = ki;
}

void BLDCMotorFoC::set_current_pi(const double& kp, const double& ki) {
  kp_c = kp;
  ki_c = ki;
}

void BLDCMotorFoC::set_load(const double TL) {
  if(std::isnan(TL)) {
    Tload = 0.0f;
  } else {
    Tload = TL;
  }
}

void BLDCMotorFoC::set_target_ang_veloc(const double omega_target) {
  omega_t = omega_target;
}

double BLDCMotorFoC::target_ang_veloc() {
  return omega_t;
}

double BLDCMotorFoC::getIdSat() const {
  return id_sat;
}

double BLDCMotorFoC::getIqSat() const {
  return iq_sat;
}

double BLDCMotorFoC::getShaftTorque(const nowtech::Vec4& y0, const double epsilon_m) const {
  /// State variables from y0
  double id = y0(0);
  double iq = y0(1);
  double omega = y0(2); // mechanical angular vel.
  // Calculate TE electromech. torque
  double TE = (3.0 / 2.0) * p * (lpm * iq + (Ld - Lq) * id * iq);
  double shaft_torque = TE - J * epsilon_m - beta * omega;
  return shaft_torque;
}

/* Utility functions */

nowtech::Vec2 ClarkeTf(nowtech::Vec3 iabc) {
  return nowtech::Vec2({(2.0 * iabc[0] - iabc[1] - iabc[2]) / 3.0, (iabc[2] - iabc[3]) / sqrt3});
}

nowtech::Vec3 InvClarkeTf(nowtech::Vec2 ialphabeta) {
  return nowtech::Vec3({ialphabeta[0],
               (-ialphabeta[0] + sqrt(3.0) * ialphabeta[1]) / 2.0,
               (-ialphabeta[0] - sqrt(3.0) * ialphabeta[1]) / 2.0});
}

nowtech::Vec2 ParkTf(nowtech::Vec2 iab, double the) {
  return nowtech::Vec2({iab[0] * cos(the) + iab[1] * sin(the), -iab[0] * sin(the) + iab[1] * cos(the)});
}

nowtech::Vec2 InvParkTf(nowtech::Vec2 idq, double th) {
  return nowtech::Vec2({idq[0] * cos(th) - idq[1] * sin(th), idq[0] * sin(th) + idq[1] * cos(th)});
}

nowtech::Vec2 ClarkeParkTf(nowtech::Vec3 iabc, double the) {
  return nowtech::Vec2(
      {2.0 * (iabc[0] * cos(the) + iabc[1] * cos(the - 2.0 * M_PI / 3.0) + iabc[2] * cos(the + 2.0 * M_PI / 3.0)) / 3.0,
       -2.0 * (iabc[0] * sin(the) + iabc[1] * sin(the - 2.0 * M_PI / 3.0) + iabc[2] * sin(the + 2.0 * M_PI / 3.0))
           / 3.0});
}

double saturate(double value, const double& absmax) {
  if (value < -absmax) {
    return -absmax;
  }
  if (value > absmax) {
    return absmax;
  }
  return value;
}