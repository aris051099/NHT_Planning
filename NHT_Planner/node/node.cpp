#ifndef NODE    
    #define NODE
    #include "node.h"
    void node::setParent(node* incoming)
    {
        parent = incoming; 
    }
    node* node::getParent()
    {
        return parent;
    }
    const Xstate& node::getconstXstate() const
    {
        return reached_state;
    }
    Xstate& node::getXstate() 
    {
        return reached_state;
    }
    Ustate& node::getUstate()
    {
        return u_control;
    }
    void node::setXstate(Xstate& x)
    {
        reached_state = x;
    }
    void node::setUstate(Ustate& u)
    {
        u_control = u;
    }
    void node::calc_f(double inc_g, double inc_h)
    {
        this->g = inc_g;
        this->h = inc_h;
        this->f = inc_g + inc_h;
    }
    void node::calc_f()
    {
        this->f = this->g + this-> h;
    }
    void node::update(node* parent,Xstate new_x,Ustate new_u,double new_g)
    {
        this->setParent(parent);
        this->reached_state = new_x;
        this->u_control = new_u;
        this->g = new_g;
    }
#endif 