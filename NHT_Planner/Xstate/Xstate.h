/** The following code allows to integrate forward in time a dynamics state space model
 * The A,B matrices are derived from MATLAB. The discretization methods used was the ZOH
 * TODO:
 * Separate integration function into a different file. Dynamics.h should only be for entering A,B,C,D matrices
 * Create a distance function between states (Euclidean for now its fine) 
 * Create threshold for vicinity 
 * Create all the implicit planning thing 
*/
#pragma once
#include <iostream> 
#include <math.h>
#include <algorithm>
#include <chrono>
#include <stdlib.h>
#include <vector>
#include "state_template.h"
#define PI 3.141592654


class Ustate : public state<double,2> 
{
  private:
  double t_prop = 0;
  public:
    Ustate();
    Ustate(double u1,double u2);
    Ustate(double u1,double u2,double t_prop);
    Ustate& operator=(const Ustate& incoming)
    {
      state_elem[0] = incoming[0];
      state_elem[1] = incoming[1];
      this->t_prop = incoming.t_prop;

      return *this;
    }
    void set_tprop(double t)
    {
      this->t_prop = t;
    }
    double get_tprop()
    {
      return this->t_prop;
    }
    bool operator==(const Ustate& incoming)
    {
      for(int i = 0 ; i < arr_size; ++i)
      {
        if(state_elem[i] != incoming[i])
        {
            return false;
        }
      }
      return true;
    }
    bool operator !=(const Ustate& incoming)
    {
      for(int i = 0 ; i < arr_size; ++i)
      {
        if(state_elem[i] != incoming[i])
        {
            return true;
        }
      }
      return false;
    }
    void setState(double u1, double u2, double inc_t)
    {
        state_elem[0] = u1;
        state_elem[1] = u2;
        this->t_prop = inc_t;
    }

    Ustate operator+(const Ustate& other) const {
        Ustate result;
        for (int i = 0; i < arr_size; ++i) {
            result.state_elem[i] = this->state_elem[i] + other.state_elem[i];
        }
        return result;
    }

    // Overload the * operator for multiplication with a constant
    Ustate operator*(double scalar) const {
        Ustate result;
        for (int i = 0; i < arr_size; ++i) {
            result.state_elem[i] = this->state_elem[i] * scalar;
        }
        return result;
    }
};

class Xstate : public state<double,4>
{
  public:

    double map_coords[2] = {0,0};
    int state=-1; 

    Xstate();

    Xstate(double x1,double x2,double x3,double x4);

    Xstate(const Xstate& inc);

    Xstate& operator=(const Xstate& incoming)
    {
      state_elem[0] = incoming[0];
      state_elem[1] = incoming[1];
      state_elem[2] = incoming[2];
      state_elem[3] = incoming[3];
      this-> map_coords[0] = incoming.map_coords[0];
      this-> map_coords[1] = incoming.map_coords[1];
      this->state = incoming.state;
      return *this;
    }
    bool operator==(const Xstate& incoming) const
    {
      for(int i = 0 ; i < arr_size; ++i)
      {
        if(state_elem[i] != incoming[i])
        {
            return false;
        }
      }
      return true;
    }
    bool operator !=(const Xstate& incoming)
    {
      for(int i = 0 ; i < arr_size; ++i)
      {
        if(state_elem[i] != incoming[i])
        {
            return true;
        }
      }
      return false;
    }

        // Overload the + operator
    Xstate operator+(const Xstate& other) const {
        Xstate result;
        for (int i = 0; i < arr_size; ++i) {
            result.state_elem[i] = this->state_elem[i] + other.state_elem[i];
        }
        return result;
    }

    // Overload the * operator for multiplication with a constant
    Xstate operator*(double scalar) const {
        Xstate result;
        for (int i = 0; i < 4; ++i) {
            result.state_elem[i] = this->state_elem[i] * scalar;
        }
        return result;
    }


    inline void sum(Xstate inc_x)
    {
      state_elem[0] = state_elem[0] + inc_x[0];
      state_elem[1] = state_elem[1] + inc_x[1];
      state_elem[2] = state_elem[2] + inc_x[2];
      state_elem[3] = state_elem[3] + inc_x[3];
    }     

    inline void subs(Xstate inc_x)
    {
      state_elem[0] = state_elem[0] - inc_x[0];
      state_elem[1] = state_elem[1] - inc_x[1];
      state_elem[2] = state_elem[2] - inc_x[2];
      state_elem[3] = state_elem[3] - inc_x[3];
    }    
    inline void const_multiply(double k)
    {
      state_elem[0] = k*state_elem[0];
      state_elem[1] = k*state_elem[1];
      state_elem[2] = k*state_elem[2];
      state_elem[3] = k*state_elem[3];
    }
    inline void setState(double x1, double x2, double x3, double x4)
    {
        state_elem[0] = x1;
        state_elem[1] = x2;
        state_elem[2] = x3;
        state_elem[3] = x4;
    }
    inline void setState(const Xstate& inc)
    {
        state_elem[0] = inc[0];
        state_elem[1] = inc[1];
        state_elem[2] = inc[2];
        state_elem[3] = inc[3];

        map_coords[0] = inc.map_coords[0];
        map_coords[1] = inc.map_coords[1];

        this->state = inc.state;
    }
};