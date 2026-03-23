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
    static constexpr int N_DIM = 4;

    node* root;

    double squared_distance(const Xstate& a, const Xstate& b) const;

    void nearest_neighbor(node* Knode, const Xstate& target,
                          double& min_distance, node*& nearest, double r_sq) const;

    void insert(node*& Knode, node* q_new, int depth) const;

    void cleanup(node* n);
};
