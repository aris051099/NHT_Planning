#ifndef KDTREE
#define KDTREE
#include "KDtree.h"

void KDTree::Insert(node* q_new)
{
    ++size;
    insert(root, q_new, 0);
}

void KDTree::cleanUp()
{
    if(root != nullptr)
    {
        cleanup(root);
        root = nullptr;
    }
    size = 0;
}

node* KDTree::nearest_neighbor(const Xstate& target, double r) const
{
    double min_distance = std::numeric_limits<double>::max();
    node*  nearest      = nullptr;
    nearest_neighbor(root, target, min_distance, nearest, r * r);
    return nearest;
}

double KDTree::squared_distance(const Xstate& a, const Xstate& b) const
{
    double dx = a[0] - b[0];
    double dy = a[1] - b[1];
    double dz = a[2] - b[2];
    double dw = a[3] - b[3];
    return dx * dx + dy * dy + dz * dz + dw * dw;
}

void KDTree::nearest_neighbor(node* Knode, const Xstate& target,
                              double& min_distance, node*& nearest, double r_sq) const
{
    if(Knode == nullptr) return;

    double distance = squared_distance(Knode->getXstate(), target);

    if(distance <= r_sq && distance < min_distance)
    {
        min_distance = distance;
        nearest      = Knode;
    }

    int    axis      = Knode->axis % N_DIM;
    double axis_diff = target[axis] - Knode->reached_state[axis];

    node* first_child  = axis_diff <= 0 ? Knode->left : Knode->right;
    node* second_child = axis_diff <= 0 ? Knode->right : Knode->left;

    nearest_neighbor(first_child, target, min_distance, nearest, r_sq);

    if(axis_diff * axis_diff < min_distance)
    {
        nearest_neighbor(second_child, target, min_distance, nearest, r_sq);
    }
}

void KDTree::insert(node*& Knode, node* q_new, int depth) const
{
    if(Knode == nullptr)
    {
        Knode = q_new;
        Knode->axis = depth % N_DIM;
        return;
    }

    int  axis   = depth % N_DIM;
    bool goLeft = q_new->getXstate()[axis] < Knode->reached_state[axis];

    if(goLeft)
        insert(Knode->left, q_new, depth + 1);
    else
        insert(Knode->right, q_new, depth + 1);
}

void KDTree::cleanup(node* n)
{
    if(n == nullptr) return;
    cleanup(n->left);
    cleanup(n->right);
    delete n;
}

#endif
