#pragma once
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <Xstate.h>

// Owns the random engine and all sampling distributions used by the planner.
// Separating sampling here makes the random state explicit and testable.
class Sampler
{
public:
    std::default_random_engine gen;

    Sampler()
    {
        auto seed = (unsigned int)std::chrono::system_clock::now().time_since_epoch().count();
        gen.seed(seed);
        std::cout << "Sampler seed: " << seed << std::endl;
    }

    // Random control for general exploration.
    Ustate sample_control()
    {
        Ustate u;
        u[0] = rand_u_vel(gen) / 100.0;
        u[1] = rand_u_ang_vel(gen);
        u.set_tprop(rand_t_prop(gen));
        return u;
    }

    // Tighter control sampling used when the tree is near the goal.
    Ustate sample_near_goal_control()
    {
        Ustate u;
        u[0] = near_rand_u_vel(gen) / 100.0;
        u[1] = near_rand_u_ang_vel(gen);
        u.set_tprop(rand_t_prop(gen));
        return u;
    }

    // Uniform random state over the map space [0,100) x [0,100).
    Xstate sample_random_state()
    {
        Xstate x;
        x.setState(rand_map_x(gen) / 10.0, rand_map_y(gen) / 10.0, rand_theta(gen), 0.0);
        return x;
    }

    // Random state within a window of size quad around the goal.
    Xstate sample_near_goal_state(double gx, double gy, int quad)
    {
        std::uniform_int_distribution<int> nx((int)gx - quad, (int)gx + quad);
        std::uniform_int_distribution<int> ny((int)gy - quad, (int)gy + quad);
        Xstate x;
        x.setState((double)nx(gen), (double)ny(gen), rand_theta(gen), 0.0);
        return x;
    }

    // Sample a probability value in [0, 1).
    double sample_prob() { return dist_prob(gen); }

private:
    static constexpr double PI = 3.141592653589793;

    std::uniform_real_distribution<double> rand_t_prop          {0.0,  8.0};
    std::uniform_real_distribution<double> rand_u_ang_vel        {-PI,  PI};
    std::uniform_int_distribution<int>     rand_u_vel            {10,   95};
    std::uniform_int_distribution<int>     near_rand_u_vel       {10,   50};
    std::uniform_real_distribution<double> near_rand_u_ang_vel   {-2.0, 2.0};
    std::uniform_real_distribution<double> dist_prob             {0.0,  1.0};
    std::uniform_real_distribution<double> rand_theta            {-PI,  PI};
    std::uniform_int_distribution<int>     rand_map_x            {0,    999};
    std::uniform_int_distribution<int>     rand_map_y            {0,    999};
};
