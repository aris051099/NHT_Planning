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
  public:
    Ustate();
    Ustate(double u1,double u2);
    Ustate& operator=(const Ustate& incoming)
    {
      state_elem[0] = incoming[0];
      state_elem[1] = incoming[1];

      return *this;
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

    void propagate(Xstate& x_old, Ustate& u_control, double prop_time)
    {
      auto x_k = x_old.getPointer();
      auto u_k = u_control.getPointer();
      for(int i = 0; i < sec2msec(prop_time) ; ++i)
      {
        // printf("Iteration %d \n",i);
        // x_k = din_sytem.A_dt*x_k + din_sytem.B_dt*u_k;
        state_elem[0] = x_k[0] + 0.007592*x_k[1] + 0.001579*u_k[0]; 
        state_elem[1] = 0.5606*x_k[1] + 0.2882*u_k[0];
        state_elem[2] = x_k[2] + 0.001705*x_k[3] + 0.008201*u_k[1];
        state_elem[3] = 0.002881*x_k[3] + 0.9858*u_k[1];
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


