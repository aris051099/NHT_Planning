/** The following code allows to integrate forward in time a dynamics state space model
 * The A,B matrices are derived from MATLAB. The discretization methods used was the ZOH
 * TODO:
 * Separate integration function into a different file. Dynamics.h should only be for entering A,B,C,D matrices
 * Create a distance function between states (Euclidean for now its fine) 
 * Create threshold for vicinity 
 * Create all the implicit planning thing 
*/

#define PI 3.141592654
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

    double map_coords[2] = {0,0};

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
      this-> map_coords[0] = incoming.map_coords[0];
      this-> map_coords[1] = incoming.map_coords[1];
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

    void sum(Xstate inc_x)
    {
      state_elem[0] = state_elem[0] + inc_x[0];
      state_elem[1] = state_elem[1] + inc_x[1];
      state_elem[2] = state_elem[2] + inc_x[2];
      state_elem[3] = state_elem[3] + inc_x[3];
    }     

    void subs(Xstate inc_x)
    {
      state_elem[0] = state_elem[0] - inc_x[0];
      state_elem[1] = state_elem[1] - inc_x[1];
      state_elem[2] = state_elem[2] - inc_x[2];
      state_elem[3] = state_elem[3] - inc_x[3];
    }    
    void const_multiply(double k)
    {
      state_elem[0] = k*state_elem[0];
      state_elem[1] = k*state_elem[1];
      state_elem[2] = k*state_elem[2];
      state_elem[3] = k*state_elem[3];
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

  map_coords[0] = inc.map_coords[0];
  map_coords[1] = inc.map_coords[1];
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

bool check_collision(Xstate& x_prop, double *map, int x_size,int y_size) 
{
  auto c_x = std::round(x_prop[0]);
  auto c_y = std::round(x_prop[1]);
  // double coords[2] = {std::round((x_prop[0]*cos(x_prop[3]))/0.01),std::round((x_prop[0]*sin(x_prop[3]))/0.01)};
  if(c_x> 0.0 && c_x < x_size && c_y > 0.0 && c_y < y_size)
  {
    int map_idx = (y_size-c_y-1)*x_size + c_x;
    if(map[map_idx] == 1.0 )
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

Xstate propagate(Xstate i_x_k, Ustate u_k,double *map, int x_size, int y_size)
{
  /*
    The present function propagate the dynamics based on a specific input. In addition,
    a collision checking method is included. Based on a map, make sure it does not get out of bounds
    However I need to include, the absolute displacement + relative displacement. 
    
    After propagating one step, we calculate the (x,y) coordinate and make sure the cell is not 1;
    NOTES: This will also depend on the resolution of the map and the resolution for rendering. Be aware
  */
      Xstate x_prop(i_x_k); 
      Xstate x_k(i_x_k);
      double h = 0.01; // Step_time 
      double prop_time = u_k.get_tprop();
      if(x_k[2] < 0)
      {
        x_k[2] = 2*PI - x_k[2];
      }
      for(int i = 0; i < sec2msec(prop_time) ; ++i)
      {
        x_prop[0] = x_k[0] + u_k[0]*cos(x_k[2])*h;
        x_prop[1] = x_k[1] + u_k[0]*sin(x_k[2])*h;
        x_prop[2] = x_k[2] + u_k[1]*h;
        x_prop[3] = 0;

        // double diff_x = x_prop[0]-x_k[0];
        // double diff_angle = x_prop[3]-x_k[3];
        // x_prop.map_coords[0] = diff_x*cos(x_prop[2]) + x_k.map_coords[0]; //Relative displacement + absolute
        // x_prop.map_coords[1] = diff_x*sin(x_prop[2]) + x_k.map_coords[1]; //Relative displacement + aboslute
        // std::cout<< x_prop[0] << " , " << x_prop[1] << ", " << x_prop[2] << std::endl;
        // x_k  = x_prop;
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

// Xstate f_x(Xstate inc_x , Ustate inc_u)
// {
//   double h = 0.01;
// }

