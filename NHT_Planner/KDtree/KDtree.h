#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <Xstate.h>
// #include "node.h"

Xstate x(1.0,1.0,1.0,1.0);
class KDnode 
{
public:
    Xstate x_s;
    KDnode* left;
    KDnode* right;
    int axis;

    KDnode(const Xstate& p, int axis) : x_s(p), left(nullptr), right(nullptr), axis(axis) {}
};

struct AxisComparator 
{
    explicit AxisComparator(int axis) : axis(axis) {}

    bool operator()(const Xstate& a, const Xstate& b) const {
        return (axis == 0) ? a[0] < b[0] :
               (axis == 1) ? a[1] < b[1] :
               (axis == 2) ? a[2] < b[2] :
                            a[3] < b[3];
    }

    int axis;
};

class KDTree 
{
public:
    KDTree() : root(nullptr) {}

    void build(std::vector<Xstate>& Xstates) {
        root = build(Xstates.begin(), Xstates.end(), 0);
    }

    Xstate nearest_neighbor(const Xstate& target) const {
        double min_distance = std::numeric_limits<double>::max();
        Xstate nearest_Xstate = target;
        nearest_neighbor(root, target, min_distance, nearest_Xstate);
        return nearest_Xstate;
    }
private:

    KDnode* root;

    using XstateIter = std::vector<Xstate>::iterator;

    KDnode* build(XstateIter begin, XstateIter end, int depth) 
    {
        if (begin == end) return nullptr;

        size_t size = std::distance(begin, end);
        int axis = depth % 4;

        AxisComparator comp(axis);

        std::nth_element(begin, begin + size / 2, end, comp);

        auto mid = begin + size / 2;
        KDnode* Knode = new KDnode(*mid, axis);
        Knode->left = build(begin, mid, depth + 1);
        Knode->right = build(mid + 1, end, depth + 1);

        return Knode;
    }


    double squared_distance(const Xstate& a, const Xstate& b) const {
        double dx = a[0] - b[0];
        double dy = a[1] - b[1];
        double dz = a[2] - b[2];
        double dw = a[3] - b[3];
        return dx * dx + dy * dy + dz * dz + dw * dw;
    }

    void nearest_neighbor(KDnode* Knode, const Xstate& target, double& min_distance, Xstate& nearest_Xstate) const 
    {
        if (!Knode) return;

        double distance = squared_distance(Knode->x_s, target);
        if (distance < min_distance) {
            min_distance = distance;
            nearest_Xstate = Knode->x_s;
        }

        double axis_diff = 0;
        int axis = Knode->axis;
        switch (axis) {
            case 0:
                axis_diff = target[0] - Knode->x_s[0];
                break;
            case 1:
                axis_diff = target[1] - Knode->x_s[1];
                break;
            case 2:
                axis_diff = target[2] - Knode->x_s[2];
                break;
            case 3:
                axis_diff = target[3] - Knode->x_s[3];
                break;
        }
        KDnode* first_child = axis_diff <= 0 ? Knode->left : Knode->right;
        KDnode* second_child = axis_diff <= 0 ? Knode->right : Knode->left;

        nearest_neighbor(first_child, target, min_distance, nearest_Xstate);

        if (axis_diff * axis_diff < min_distance) {
            nearest_neighbor(second_child, target, min_distance, nearest_Xstate);
        }
    }
};

void insert(KDnode* Knode, const Xstate& Xstate, int depth) 
{
    if(Knode == nullptr)
    {
        std::cout<<"Root is null" << std::endl;
        return;
    }
    if (!Knode) {
        return;
    }

    int axis = depth % 4;
    bool goLeft = false;

    if (axis == 0) {
        goLeft = Xstate[0] < Knode->x_s[0];
    } else if (axis == 1) {
        goLeft = Xstate[1] < Knode->x_s[1];
    } else if (axis == 2) {
        goLeft = Xstate[2] < Knode->x_s[2];
    } else {
        goLeft = Xstate[3] < Knode->x_s[3];
    }

    KDnode*& nextKnode = goLeft ? Knode->left : Knode->right;

    if (!nextKnode) {
        nextKnode = new KDnode(Xstate, (depth + 1) % 4);
    } else {
        insert(nextKnode, Xstate, depth + 1);
    }
}

       
