#ifndef XSTATE    
    #define XSTATE
    #include "Xstate.h"
    Ustate::Ustate()
    {
    state_elem[0] = 0;
    state_elem[1] = 0;
    }

    Ustate::Ustate(double u1, double u2)
    {
    state_elem[0] = u1;
    state_elem[1] = u2;
    }

    Ustate::Ustate(double u1, double u2, double inc_t)
    {
    state_elem[0] = u1;
    state_elem[1] = u2;
    this->t_prop = inc_t;
    }

    std::ostream& operator<<(std::ostream& os, const Ustate& u)
    {
    // auto x_pointer = x.getPointer();
    os << " " << u[0] << " " << std::endl;
    os << " " << u[1] << " "<< std::endl;
        // os << dt.mo << '/' << dt.da << '/' << dt.yr;
        return os;
    }
    Xstate::Xstate()
    {
    state_elem[0] = 0;
    state_elem[0] = 0;
    state_elem[0] = 0;
    state_elem[0] = 0; 
    }

    Xstate::Xstate(double x1, double x2, double x3, double x4)
    {
    state_elem[0] = x1;
    state_elem[1] = x2;
    state_elem[2] = x3;
    state_elem[3] = x4;
    }

    Xstate::Xstate(const Xstate& inc)
    {
    state_elem[0] = inc[0];
    state_elem[1] = inc[1];
    state_elem[2] = inc[2];
    state_elem[3] = inc[3];

    map_coords[0] = inc.map_coords[0];
    map_coords[1] = inc.map_coords[1];

    this->state = inc.state;
    }

    std::ostream& operator<<(std::ostream& os, const Xstate& x)
    {
    // auto x_pointer = x.getPointer();
    os << " " << x[0] << " " << std::endl;
    os << " " << x[1] << " "<< std::endl;
    os << " " << x[2] << " "<< std::endl;
    os << " " << x[3] << " "<< std::endl; 
        // os << dt.mo << '/' << dt.da << '/' << dt.yr;
        return os;
    }

    // bool check_collision(Xstate& x_prop, double *map, int x_size,int y_size) 
    // {
    //   auto c_x = x_prop[0];
    //   auto c_y = x_prop[1];
    //   double theta = x_prop[2];

    //   for(double p_x = c_x-0.990/2.0; p_x < c_x+0.990/2.0;p_x+=0.1)
    //     {
    //       for(double p_y = c_y-0.67/2.0; p_y < c_y+0.67/2.0;p_y+=0.1)
    //       {
    //         double r_x = c_x + p_x*cos(theta) + p_y*sin(theta);
    //         double r_y = c_y + p_x*sin(theta) + p_y*cos(theta);
    //         if(r_x> 0.0 && r_x < 1.0*x_size && r_y > 0.0 && r_y < 1.0*y_size)
    //         {
    //           int map_idx = (y_size-std::round(r_y)-1)*x_size + std::round(r_x);
    //           if(map[map_idx] == 1.0 )
    //           {
    //             return false;
    //           }
    //         }
    //         else
    //         {
    //           return false;
    //         }
    //       }
    //     }
    //     return true;
    // }

    // Xstate f_x(Xstate inc_x , Ustate inc_u)
    // {
    //   double h = 0.01;
    // }

    // #include <cmath>
    // #include <iostream>
    // #include <vector>
    // #include <utility>

    // const double kEpsilon = 1e-6;

    // double fresnel_integral(double x, double k) {
    //     int num_intervals = 1000;
    //     double delta = x / num_intervals;
    //     double sum = 0;

    //     for (int i = 0; i < num_intervals; ++i) {
    //         double xi = i * delta;
    //         double xi_plus_1 = (i + 1) * delta;
    //         double fi = std::cos(0.5 * k * xi * xi);
    //         double fi_plus_1 = std::cos(0.5 * k * xi_plus_1 * xi_plus_1);

    //         sum += (fi + fi_plus_1) / 2 * delta;
    //     }

    //     return sum;
    // }

    // std::pair<double, double> clothoid(double s, double k) {
    //     double x = fresnel_integral(s, k);
    //     double y = fresnel_integral(s, -k);
    //     return {x, y};
    // }

    // std::vector<std::pair<double, double>> generate_clothoid_path(double s0, double s1, double k, int num_points) {
    //     std::vector<std::pair<double, double>> path;
    //     double delta_s = (s1 - s0) / (num_points - 1);

    //     for (int i = 0; i < num_points; ++i) {
    //         double s = s0 + i * delta_s;
    //         path.push_back(clothoid(s, k));
    //     }

    //     return path;
    // }

    // std::vector<std::pair<double, double>> compute_control_inputs(const std::vector<std::pair<double, double>>& path, double dt) {
    //     std::vector<std::pair<double, double>> control_inputs;

    //     for (size_t i = 1; i < path.size(); ++i) {
    //         double dx = path[i].first - path[i - 1].first;
    //         double dy = path[i].second - path[i - 1].second;
    //         double distance = std::sqrt(dx * dx + dy * dy);
    //         double linear_velocity = distance / dt;

    //         double delta_theta = std::atan2(dy, dx) - std::atan2(path[i - 1].second, path[i - 1].first);
    //         double angular_velocity = delta_theta / dt;

    //         control_inputs.push_back({linear_velocity, angular_velocity});
    //     }

    //     return control_inputs;
    // }

    // std::vector<std::pair<double, double>> compute_control_inputs(const std::vector<std::pair<double, double>>& path, double dt, double max_linear_velocity, double max_angular_velocity) {
    //     std::vector<std::pair<double, double>> control_inputs;

    //     for (size_t i = 1; i < path.size(); ++i) {
    //         double dx = path[i].first - path[i - 1].first;
    //         double dy = path[i].second - path[i - 1].second;
    //         double distance = std::sqrt(dx * dx + dy * dy);
    //         double linear_velocity = distance / dt;
    //         double delta_theta = std::atan2(dy, dx) - std::atan2(path[i - 1].second, path[i - 1].first);
    //         double angular_velocity = delta_theta / dt;

    //         // Clamp linear and angular velocities to their respective limits
    //         linear_velocity = std::min(linear_velocity, max_linear_velocity);
    //         angular_velocity = std::min(std::max(angular_velocity, -max_angular_velocity), max_angular_velocity);

    //         control_inputs.push_back({linear_velocity, angular_velocity});
    //     }

    //     return control_inputs;
    // }

    // class Clothoid {
    // public:
    //     Clothoid(double k0, double k, double s0, double s1) : k0_(k0), k_(k), s0_(s0), s1_(s1) {}

    //     std::vector<std::pair<double, double>> sample_points(double ds) {
    //         std::vector<std::pair<double, double>> points;

    //         for (double s = s0_; s <= s1_; s += ds) {
    //             double x = fresnel_integral(s * std::sqrt(0.5 * k_));
    //             double y = fresnel_integral((s + k0_) * std::sqrt(0.5 * k_));
    //             points.push_back({x, y});
    //         }

    //         return points;
    //     }

    // private:
    //     double fresnel_integral(double t) {
    //         // This function computes the Fresnel integral, which is used to calculate the x and y coordinates of the clothoid.
    //         // You can replace this function with a more accurate or efficient implementation, if desired.
    //         const int num_terms = 10;
    //         double sum = 0;
    //         double coef = 1;

    //         for (int n = 0; n < num_terms; ++n) {
    //             sum += coef * std::pow(t, 4 * n + 1) / (4 * n + 1);
    //             coef = -coef / (2 * n + 1) / (2 * n + 2);
    //         }

    //         return sum;
    //     }

    //     double k0_; // initial curvature
    //     double k_;  // curvature rate
    //     double s0_; // initial arc length
    //     double s1_; // final arc length
    // };


    // int main() 
    // {
    //     double s0 = 0;
    //     double s1 = 10;
    //     double k = 1;
    //     int num_points = 100;
    //     double dt = 0.1;  // Time step between path points

    //     std::vector<std::pair<double, double>> path = generate_clothoid_path(s0, s1, k, num_points);
    //     std::vector<std::pair<double, double>> control_inputs = compute_control_inputs(path, dt);

    //     for (const auto& control_input : control_inputs) {
    //         std::cout << "Linear velocity: " << control_input.first << ", Angular velocity: " << control_input.second << std::endl;
    //     }
    // }
    //     return 

#endif 
