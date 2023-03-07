/** The following code allows to integrate forward in time a dynamics state space model
 * The A,B matrices are derived from MATLAB. The discretization methods used was the ZOH
 * TODO:
 * Separate integration function into a different file. Dynamics.h should only be for entering A,B,C,D matrices
 * Create a distance function between states (Euclidean for now its fine) 
 * Create threshold for vicinity 
 * Create all the implicit planning thing 
*/
int sec2msec(double sec)
{
  return sec*100;
}

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
    
};

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

class Xstate : public state<double,4>
{
  public:

    Xstate();

    Xstate(double x1,double x2,double x3,double x4);

    Xstate(const Xstate& inc);

    void propagate(Xstate x_k, Ustate& u_k)
    {
      double prop_time = u_k.get_tprop();
      for(int i = 0; i < sec2msec(prop_time) ; ++i)
      {
        // printf("Iteration %d \n",i);
        // x_k = din_sytem.A_dt*x_k + din_sytem.B_dt*u_k;
        state_elem[0] = x_k[0] + 0.007592*x_k[1] + 0.001579*u_k[0]; 
        state_elem[1] = 0.5606*x_k[1] + 0.2882*u_k[0];
        state_elem[2] = x_k[2] + 0.001705*x_k[3] + 0.008201*u_k[1];
        state_elem[3] = 0.002881*x_k[3] + 0.9858*u_k[1];
        x_k[0] = state_elem[0];
        x_k[1] = state_elem[1];
        x_k[2] = state_elem[2];
        x_k[3] = state_elem[3];
        // std::cout << x_k << std::endl;
      }
    };

    Xstate& operator=(const Xstate& incoming)
    {
      state_elem[0] = incoming[0];
      state_elem[1] = incoming[1];
      state_elem[2] = incoming[2];
      state_elem[3] = incoming[3];
      return *this;
    }
    bool operator==(const Xstate& incoming)
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
};

Xstate::Xstate()
{
  state_elem[0] = 0;
  state_elem[0] = 0;
  state_elem[0] = 0;
  state_elem[0] = 0; 
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

bool check_collision(Xstate x_prop, double *map, int x_size,int y_size)
{
  double coords[2] = {std::round((x_prop[0]*cos(x_prop[3]))/0.01),std::round((x_prop[0]*sin(x_prop[3]))/0.01)};
  if(coords[0] > 0 && coords[0] < x_size && coords[1] > 0 && coords[1] < y_size)
  {
    int map_idx = coords[1] * x_size + coords[0];
    if(map[map_idx] == 1 )
    {
      return false;
    }
    else
    {
      return true;
    }
  }
  else
  {
    return false;
  }
}

Xstate propagate(Xstate x_k, Ustate u_k,double *map, int x_size, int y_size)
{
  /*
    The present function propagate the dynamics based on a specific input. In addition,
    a collision checking method is included. Based on a map, make sure it does not get out of bounds
    However I need to include, the absolute displacement + relative displacement. 
    
    After propagating one step, we calculate the (x,y) coordinate and make sure the cell is not 1;
    NOTES: This will also depend on the resolution of the map and the resolution for rendering. Be aware
  */
      Xstate x_prop; 
      double prop_time = u_k.get_tprop();
      for(int i = 0; i < sec2msec(prop_time) ; ++i)
      {
        // printf("Iteration %d \n",i);
        // x_k = din_sytem.A_dt*x_k + din_sytem.B_dt*u_k;
        x_prop[0] = x_k[0] + 0.007592*x_k[1] + 0.001579*u_k[0]; 
        x_prop[1] = 0.5606*x_k[1] + 0.2882*u_k[0];
        x_prop[2] = x_k[2] + 0.001705*x_k[3] + 0.008201*u_k[1];
        x_prop[3] = 0.002881*x_k[3] + 0.9858*u_k[1];
        if(check_collision(x_prop,map,x_size,y_size))
        {
          x_k = x_prop;
        }
        else
        {
          return x_k;
        }
      }
      return x_prop; 
}


Xstate gc2gs(double x, double y , double angle,double x_s, double y_s, double angle_s) // Converts goal coordinate, to a goal state based on the Start state
{
  /*
  There are a couple of questions when designing this function:
  -The idea is that, by giving (x,y,angle) coordinates on the map, you can create a state[x,xdot,angle,angledot] for the 
  algorithm to reach
  -However, based on my logic, the linear displacement (x) can be both positive and negative
    -The question then is, how can I design such a state with a negative linear displacement?
    -My first method just involves calculating the euclidean distance between (x_g,y_g) and (x_s,y_s) however, this only 
    encodes a "forward" distance or movement. 
    -Should I just limit my algorithm to move forward? 
  -The difference in angles is fine. And the velocities at the goal are assumed to be 0. 
  NOTES: For now, we are just considering, "forward" goals. 
  */
  double goal_x = sqrt(pow(x-x_s,2) + pow(y-y_s,2)); //Euclidean distance from start to goal 
  // double diff_x = x-x_s; 
  // double diff_y = y-y_s; 
  double goal_angle = angle - angle_s;
  if(goal_angle < 0)
  {
    goal_angle = 3.141592654 + goal_angle; 
  }
  Xstate x_goal(goal_x,0,goal_angle,0);

  return x_goal;
}

