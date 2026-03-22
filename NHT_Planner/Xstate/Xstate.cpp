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
    state_elem[1] = 0;
    state_elem[2] = 0;
    state_elem[3] = 0;
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

#endif
