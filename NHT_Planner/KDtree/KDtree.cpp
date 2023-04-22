#ifndef KDTREE    
    #define KDTREE
    #include "KDtree.h"

    void KDTree::cleanUp()
    {
        size = 0;
        if(root != nullptr)
        {
            cleanup(root);
            root = nullptr;
        }

    }
    node* KDTree::nearest_neighbor(const Xstate& target,double r) const 
    {
        double min_distance = std::numeric_limits<double>::max();
        node* nearest_Xstate = nullptr;
        nearest_neighbor(root, target, min_distance, nearest_Xstate,r);
        return nearest_Xstate;
    }

    node* KDTree::getRoot()
    {
        return root;
    }
    
    node* KDTree::find(node* inc_q)
    {
        return find_recursive(root,inc_q,0);
    }

    bool KDTree::is_empty() const 
    {
        return root == nullptr;
    }
    void KDTree::setNull()
    {
        root = nullptr;
    }
    void KDTree::Insert(node* q_new)
    {
        size +=1;
        insert(root,q_new,0,nullptr);
    }

    void KDTree::removeRoot()
    {
        if(root != nullptr)
        {
            delete root;
            root = nullptr;
        }
    }
    double KDTree::squared_distance(const Xstate& a, const Xstate& b) const {
        double dx = a[0] - b[0];
        double dy = a[1] - b[1];
        double dz = a[2] - b[2];
        double dw = a[3] - b[3];
        return dx * dx + dy * dy + dz * dz + dw * dw;
    }

    void KDTree::nearest_neighbor(node* Knode,const Xstate& target, double& min_distance, node*& nearest_Xstate,double r) const 
    {

        if(Knode == nullptr) return;

        double distance = squared_distance(Knode->getXstate(), target);

        if (distance < min_distance && distance <= r)
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

        nearest_neighbor(first_child, target, min_distance, nearest_Xstate,r);

        if (axis_diff * axis_diff < min_distance) {
            nearest_neighbor(second_child, target, min_distance, nearest_Xstate,r);
        }
    }

    void KDTree::insert(node*& Knode, node* q_new, int depth,node* parent) const
    {
        if (Knode == nullptr) 
        {
            Knode = q_new;
            Knode->axis = depth;
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

    void KDTree::cleanup(node* node) 
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
    node* KDTree::find_recursive(node* Knode,const node*inc_q, int depth) 
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
        Xstate point = inc_q->getconstXstate();
        printf("%d",axis);
        bool goLeft = point[axis] < Knode->reached_state[axis];
        node* nextNode = goLeft ? Knode->left: Knode->right;
        node* foundNode = find_recursive(nextNode,inc_q,depth+1);

        return foundNode;
    }
#endif 