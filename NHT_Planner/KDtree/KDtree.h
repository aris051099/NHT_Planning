#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
// #include <Xstate.h>
#include <node.h>
// #include "node.h"
node* q = new node();

// class node 
// {
// public:
//     Xstate x_s;
//     node* left;
//     node* right;
//     int axis;

//     node(const Xstate& p, int axis) : x_s(p), left(nullptr), right(nullptr), axis(axis) {}
// };


struct AxisComparator 
{
    explicit AxisComparator(int axis) : axis(axis) {}

    bool operator()(const node* a, const node* b) const {
        return (axis == 0) ? a->reached_state[0] < b->reached_state[0] :
               (axis == 1) ? a->reached_state[1] < b->reached_state[1] :
               (axis == 2) ? a->reached_state[2] < b->reached_state[2] :
                            a->reached_state[3] < b->reached_state[3];
    }

    int axis;
};

class KDTree 
{
public:
    KDTree() : root(nullptr) {}

    node* nearest_neighbor(const Xstate& target) const 
    {
        double min_distance = std::numeric_limits<double>::max();
        node* nearest_Xstate;
        nearest_neighbor(root, target, min_distance, nearest_Xstate);
        return nearest_Xstate;
    }
    void Insert(node* q_new)
    {
        insert(root,q_new,0,nullptr);
    }

    node* find(node* inc_q)
    {
        return find_recursive(root,inc_q,0);
    }
    ~KDTree()
    {
        // cleanup(root);
        // root = nullptr;
    }

    bool is_empty() const 
    {
        return root == nullptr;
    }
    void Clear()
    {

        // cleanup(root);
    }
    void setNull()
    {
        root = nullptr;
    }

    node* getRoot()
    {
        return root;
    }
    
    void removeRoot()
    {
        if(root != nullptr)
        {
            delete root;
            root = nullptr;
        }
    }

private:

    node* root;

    using XstateIter = std::vector<node*>::iterator;

    double squared_distance(const Xstate& a, const Xstate& b) const {
        double dx = a[0] - b[0];
        double dy = a[1] - b[1];
        double dz = a[2] - b[2];
        double dw = a[3] - b[3];
        return dx * dx + dy * dy + dz * dz + dw * dw;
    }

    void nearest_neighbor(node* Knode,const Xstate& target, double& min_distance, node*& nearest_Xstate) const 
    {

        if(Knode == nullptr) return;

        double distance = squared_distance(Knode->getXstate(), target);

        if (distance < min_distance) 
        {
            min_distance = distance;
            nearest_Xstate = Knode;
        }

        double axis_diff = 0;
        int axis = Knode->axis;
        switch (axis) 
        {
            case 0:
                axis_diff = target[0] - Knode->reached_state[0];
                break;
            case 1:
                axis_diff = target[1] - Knode->reached_state[1];
                break;
            case 2:
                axis_diff = target[2] - Knode->reached_state[2];
                break;
            case 3:
                axis_diff = target[3] - Knode->reached_state[3];
                break;
        }
        node* first_child = axis_diff <= 0 ? Knode->left : Knode->right;
        node* second_child = axis_diff <= 0 ? Knode->right : Knode->left;

        nearest_neighbor(first_child, target, min_distance, nearest_Xstate);

        if (axis_diff * axis_diff < min_distance) {
            nearest_neighbor(second_child, target, min_distance, nearest_Xstate);
        }
    }

    void insert(node*& Knode, node* q_new, int depth,node* parent) const
    {
        if (Knode == nullptr) 
        {
            Knode = q_new;
            return;
        }

        int axis = depth % 4;
        bool goLeft = false;
        Xstate point = q_new->getXstate();

        if (axis == 0) {
            goLeft = point[0] < Knode->reached_state[0];
        } else if (axis == 1) {
            goLeft = point[1] < Knode->reached_state[1];
        } else if (axis == 2) {
            goLeft = point[2] < Knode->reached_state[2];
        } else {
            goLeft = point[3] < Knode->reached_state[3];
        }

        if (goLeft) {
            insert(Knode->left, q_new, depth + 1,Knode);
        } else {
            insert(Knode->right, q_new, depth + 1,Knode);
        }
    }

    void cleanup(node* node) 
    {
        if (node == nullptr) 
        {
            return;
        }

        // Recursively visit and delete left and right children
        cleanup(node->left);
        cleanup(node->right);

        // Delete the current node
        delete node;
    }
    node* find_recursive(node* Knode,const node*inc_q, int depth) 
    {
        
        if (Knode == nullptr) 
        {
            return nullptr;
        }

        if (Knode == inc_q ) 
        {
            return Knode;
        }

        int axis = depth % 4;
        Xstate point = inc_q->getXstate();
        printf("%d",axis);
        bool goLeft = point[axis] < Knode->reached_state[axis];
        node* nextNode = goLeft ? Knode->left: Knode->right;
        node* foundNode = find_recursive(nextNode,inc_q,depth+1);

        return foundNode;
    }
};

       
