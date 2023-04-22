#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
// #include <Xstate.h>
#include <node.h>

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
    KDTree() : root(nullptr) {};
    ~KDTree()
    {
        // cleanup(root);
        // root = nullptr;
    }

    node* nearest_neighbor(const Xstate& target,double r) const;
    node* find(node* inc_q);
    node* getRoot();

    void Insert(node* q_new);
    void setNull();
    void removeRoot();
    void cleanUp();
    int size = 0;

    bool is_empty() const;
private:

    node* root;

    using XstateIter = std::vector<node*>::iterator;

    double squared_distance(const Xstate& a, const Xstate& b) const;

    void nearest_neighbor(node* Knode,const Xstate& target, double& min_distance, node*& nearest_Xstate,double r) const;

    void insert(node*& Knode, node* q_new, int depth,node* parent) const;

    node* find_recursive(node* Knode,const node*inc_q, int depth);
    
    void cleanup(node* node);
};

       
