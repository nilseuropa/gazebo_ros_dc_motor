#ifndef DIFFERENTIAL_EQUATIONS_BLDCMOTOR_FIELD_ORIENTED_CONTROL_H
#define DIFFERENTIAL_EQUATIONS_BLDCMOTOR_FIELD_ORIENTED_CONTROL_H

#include "nowmath.h"
#include "rk45.h"
#include "system.h"

const double rpsTokRPM = 1000.0 * 2.0 * M_PI / 60.0; // rad/s to kiloRPM
const double kRPMtorps = 1.0 / rpsTokRPM;
const double sqrt3 = sqrt(3.0);

// Transfrorms {ia,ib,ic} to {ialpha, ibeta}
nowtech::Vec2 ClarkeTf(nowtech::Vec3 iabc);

nowtech::Vec3 InvClarkeTf(nowtech::Vec2 ialphabeta);

// Transforms {ialpha, ibeta} to {id, iq} (which is rotating frame at theta_e)
nowtech::Vec2 ParkTf(nowtech::Vec2 iab, double the);

nowtech::Vec2 InvParkTf(nowtech::Vec2 idq, double th);

// Transform {ia, ib, ic} to {id, iq} in one step. Same as applying Clarke and Park TFs
nowtech::Vec2 ClarkeParkTf(nowtech::Vec3 iabc, double the);

class BLDCMotorFoC : public nowmath::DynamicalSystemBase<nowtech::Vec4> {
private:
  double R;    /// stator phase winding resistance (Ohms)
  double kt;   /// Torque constant in SI [Nm/A]
  double Ls;   /// self inductance
  double Ms;   /// mutual inductance
  double Lm;   /// stator inductance fluctuation
  double p;    /// number of motor pole pairs (p = P/2)
  double J;    /// Rotor inertia (kg m^2).
  double beta; /// Friction coefficient (Nms / rad)
  /* Calculated params */
  double Ld;   /// d-axis winding inductance (Henry).
  double Lq;   /// q-axis winding inductance (Henry).
  double lpm;  /// (lambda_pm) permanent magnet flux linkage (Weber = Wb = V s)
  double Uref; /// Reference voltage of the BLDC motor (supply voltage)
  /* Simulation params */
  double dt;      /// Time step for control. Internal simulation time step is smaller.
  double Tload;   /// Torque load
  double Vd, Vq;  /// Voltages in d-q axes
  double omega_t; /// Desired angular veloc
  /* Control params */
  double kp_v; /// Prop. gain for velocity
  double ki_v; /// Integral gain for velocity
  double kp_c; /// Prop. gain for current
  double ki_c; /// Integral gain for current
  double Tref; /// Reference torque for speed control
  double imax; /// Current limit during MTPA
  /* Control state variables */
  double v_int_err; /// speed control integral error;
  double id_int_err;
  double iq_int_err;
  double id_sat; // Calculated required, saturated id current
  double iq_sat; // Calculated required, saturated iq current
  void speed_control_pi(const nowtech::Vec4& state);
  void foc_control_surface_pmsm(const nowtech::Vec4& state);

public:
  BLDCMotorFoC(double r,
               double kT,
               double imax,
               double ls,
               double ms,
               double lm,
               double p,
               double j,
               double beta,
               double Umax,
               double dt);
  virtual ~BLDCMotorFoC() {}
  nowtech::Vec4 f(const nowtech::Vec4& y0, const double& t) const;
  nowtech::Vec4 step(const nowtech::Vec4& state) override;
  void set_veloc_pi(const double& kp, const double& ki);
  void set_current_pi(const double& kp, const double& ki);
  void set_load(const double Tload);
  void set_target_ang_veloc(const double omega_target);
  double target_ang_veloc();
  void reset_controller();
  double getIdSat() const;
  double getIqSat() const;
  double getShaftTorque(const nowtech::Vec4& y0, const double epsilon_m = 0.0) const;

  // For debugging the PI controller:
  double getReferenceTorque() { return Tref; }
  double getIdIntegralError() { return id_int_err; }
  double getIqIntegralError() { return iq_int_err; }
  double getVoltaged() { return Vd; }
  double getVoltageq() { return Vq; }
};


#endif // DIFFERENTIAL_EQUATIONS_BLDCMOTOR_FIELD_ORIENTED_CONTROL_H
