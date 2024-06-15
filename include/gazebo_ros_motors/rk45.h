#ifndef CELL_MAPPING_CPP_RK45_H
#define CELL_MAPPING_CPP_RK45_H

#include <cstdio>
#include "nowmath.h"

namespace nowmath {

    /**
     * \brief RK45 Scheme with adaptive step-size control
     * Can be used on any system having member function f(y,t)
     */
    template <class T, class S>
    class RK45 {
    private:
        const S* sys; 	/**< Pointer to a system having member function f */
        T k1, k2, k3, k4, k5, k6, k7, c6; /**< Intermediate slopes */
        T y15, y14;		/**< Fifth- and fourth-order solutions */
        double relTol; 	/**< Relative tolerance between the two solutions */
        double stepSizeIncrement = 1.1;	/**< Step size is incremented with this factor */
        double stepSizeDecrement = 0.1;	/**< Step size is decreased with this factor */
        double relErr = 0.0;
        T internalStep(const T& y, double& t, double& h) {
            k1 = sys->f(y, t);
            k2 = sys->f(y + 1.0/5.0*h*k1, t + 1.0/5.0*h);
            k3 = sys->f(y + 3.0/40.0*h*k1 + 9.0/40.0*h*k2, t + 3.0/10.0*h);
            k4 = sys->f(y + 44.0/45.0*h*k1 - 56.0/15.0*h*k2 + 32.0/9.0*h*k3, t + 4.0/5.0*h);
            k5 = sys->f(y + 19372.0/6561.0*h*k1 - 25360.0/2187.0*h*k2 + 64448.0/6561.0*h*k3 - 212.0/729.0*h*k4, t + 8.0/9.0*h);
            k6 = sys->f(y + 9017.0/3168.0*h*k1 - 355.0/33.0*h*k2 + 46732.0/5247.0*h*k3 + 49.0/176.0*h*k4 - 5103.0/18656.0*h*k5, t + h);
            c6 = 35.0/384.0*h*k1 + 500.0/1113.0*h*k3 + 125.0/192.0*h*k4 - 2187.0/6784.0*h*k5 + 11.0/84.0*h*k6;
            k7 = sys->f(y + c6, t + h);
            y15 = y + c6;
            y14 = y + 5179.0/57600.0*h*k1 + 7571.0/16695.0*h*k3 + 393.0/640.0*h*k4 - 92097.0/339200.0*h*k5 + 187.0/2100.0*h*k6 + 1.0/40.0*h*k7;
#ifdef NOWMATH_BACKEND_ARMA
            if (norm(y15) >= 1e-20) {
                relErr = abs(norm(y15-y14)/norm(y15));
            } else {
                relErr = 2.0*relTol; // keep step size and accept step
            }
#else
            if (y15.norm() >= 1e-20) {
                relErr = abs((y15-y14).norm()/y15.norm());
            } else {
                relErr = 2.0*relTol; // keep step size and accept step
            }
#endif
            if (relErr < relTol) {
                // If true, increase internal step size and continue
                t+=h;
                h*=stepSizeIncrement;
                return y15;
            } else if (relErr < 10.0*relTol) {
                // Keep step size and continue
                t+=h;
                return y15;
            } else {
                // Decrease internal step size
                h*=stepSizeDecrement;
                return y;
            }
        }
    public:
        RK45(const S* systemPointer, double relativeTolerance) : sys(systemPointer), relTol(relativeTolerance) { }
        T step(const T& y0, const double t0, const double dt, const double h0) {
            // Uses internalStep to integrate the system from t_start = t0 to t_end = t0+dt
            double t = t0;
            double h = h0;
            T y = y0;
            while (t < t0+dt) {
                // To reach t0+dt exactly, reduce the final step size if necessary
                if (t+h > t0+dt) { h = (t0+dt-t); }
                y = internalStep(y,t,h);
            }
            return y;
        }
        T step(const T& y0, const double t0, const double dt, const double h0, const size_t maxsteps) {
            // Uses internalStep to integrate the system from t_start = t0 to t_end = t0+dt
            double t = t0;
            double h = h0;
            T y = y0;
            size_t steps = 0;
            while (t < t0+dt) {
                steps++;
                if (steps > maxsteps) {
                    // Stop integration if maximum steps exceeded
                    break;
                }
                // To reach t0+dt exactly, reduce the final step size if necessary
                if (t+h > t0+dt) { h = (t0+dt-t); }
                y = internalStep(y,t,h);
            }
            return y;
        }

    };

}

#endif //CELL_MAPPING_CPP_RK45_H