#pragma once
#include <cmath>
#include <limits>
#include <node.h>

class KDTree
{
public:
    KDTree() : root(nullptr) {}
    ~KDTree()
    {
        cleanup(root);
        root = nullptr;
    }

    void   Insert(node* q_new);
    node*  nearest_neighbor(const Xstate& target, double r) const;
    void   cleanUp();
    int    size = 0;

private:
    static constexpr int    N_DIM    = 4;
    static constexpr double WRAP_PI  = 3.141592653589793;
    static constexpr double WRAP_2PI = 6.283185307179586;

    node* root;

    // Wrap an angular difference to [-π, π].
    static double wrap_angle(double d)
    {
        while(d >  WRAP_PI) d -= WRAP_2PI;
        while(d < -WRAP_PI) d += WRAP_2PI;
        return d;
    }

    // Axis difference: plain subtraction for position (0,1), wrapped for angles (2,3).
    static double axis_difference(int axis, double a, double b)
    {
        double d = a - b;
        return (axis >= 2) ? wrap_angle(d) : d;
    }

    double squared_distance(const Xstate& a, const Xstate& b) const;

    void nearest_neighbor(node* Knode, const Xstate& target,
                          double& min_distance, node*& nearest, double r_sq) const;

    void insert(node*& Knode, node* q_new, int depth) const;

    void cleanup(node* n);
};
