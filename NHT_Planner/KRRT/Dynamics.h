#pragma once
#include <cmath>
#include <Xstate.h>

// Returns true if x is in free space, false if obstacle or out of bounds.
inline bool check_collision(const Xstate& x, const double* map, int x_size, int y_size)
{
    double c_x = x[0];
    double c_y = x[1];
    if(c_x > 0.0 && c_x < (double)x_size && c_y > 0.0 && c_y < (double)y_size)
    {
        int map_idx = (y_size - (int)std::round(c_y) - 1) * x_size + (int)std::round(c_x);
        return map[map_idx] != 1.0;
    }
    return false;
}

// Pure physics: continuous-time kinematics + numerical integration for the
// tether-constrained Husky UGV.
//
//   State  x = [px, py, theta, beta]
//   Input  u = [v, omega]
//
//   dx/dt = [v*cos(theta),  v*sin(theta),  omega,  eps*v*sin(beta) + omega]
class Dynamics
{
public:
    double h     = 0.01;   // integration timestep (s)
    double eps   = 0.085;  // tether angle / velocity coupling
    double alpha = 1.0;    // angular velocity coupling for Euler model

    // Number of integration steps for a given propagation time.
    int sec2msec(double sec) const
    {
        return (int)((std::round(sec * 100.0) / 100.0) * 100);
    }

    // Continuous-time state derivatives: xdot = f(x, u)
    Xstate f(const Xstate& x, const Ustate& u) const
    {
        Xstate xdot;
        xdot[0] = u[0] * std::cos(x[2]);
        xdot[1] = u[0] * std::sin(x[2]);
        xdot[2] = u[1];
        xdot[3] = eps * u[0] * std::sin(x[3]) + u[1];
        return xdot;
    }

    // RK4 integration for one step of size h.
    Xstate rk4step(const Xstate& x, const Ustate& u) const
    {
        Xstate k1 = f(x, u);
        Xstate k2 = f(x + k1 * (h / 2.0), u);
        Xstate k3 = f(x + k2 * (h / 2.0), u);
        Xstate k4 = f(x + k3 * h, u);
        return x + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (h / 6.0);
    }

    // Euler integration for one step of size h.
    Xstate euler_step(const Xstate& x, const Ustate& u) const
    {
        Xstate x_next(x);
        x_next[0] = x[0] + u[0] * std::cos(x[2]) * h;
        x_next[1] = x[1] + u[0] * std::sin(x[2]) * h;
        x_next[2] = x[2] + u[1] * h;
        x_next[3] = x[3] + eps * u[0] * h * std::sin(x[3]) + alpha * u[1] * h;
        return x_next;
    }

    // Propagate forward using RK4, stopping at the first collision.
    // On collision: sets x_k.state = 2 and truncates u_k.tprop to elapsed time.
    // On clean completion: sets x_k.state = 1.
    Xstate propagate(const Xstate& x0, Ustate& u_k,
                     const double* map, int x_size, int y_size) const
    {
        Xstate x_k(x0);
        int steps = sec2msec(u_k.get_tprop());
        for(int i = 0; i < steps; ++i)
        {
            Xstate x_next = rk4step(x_k, u_k);
            if(check_collision(x_next, map, x_size, y_size))
            {
                x_k = x_next;
            }
            else
            {
                x_k.state = 2;
                u_k.set_tprop(i / 100.0);
                return x_k;
            }
        }
        x_k.state = 1;
        return x_k;
    }
};
