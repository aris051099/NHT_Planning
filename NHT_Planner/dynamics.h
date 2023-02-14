/** The following code allows to integrate forward in time a dynamics state space model
 * The A,B matrices are derived from MATLAB. The discretization methods used was the ZOH
 * TODO:
 * Separate integration function into a different file. Dynamics.h should only be for entering A,B,C,D matrices
 * Create a distance function between states (Euclidean for now its fine) 
 * Create threshold for vicinity 
 * Create all the implicit planning thing 
*/
#include <math.h>
#include <algorithm>
#include <chrono>
#include <random>
#include <stdexcept>
#include <regex> // For regex and split logic
#include <iostream> // cout, endl
#include <fstream> // For reading/writing files
#include <eigen3/Eigen/Dense>

#define PI 3.141592654

unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
//1234 12345 123456 1234567 12345678
std::default_random_engine generator(seed);
std::uniform_real_distribution<double> rand_t_prop(0.0,10.0);
std::uniform_real_distribution<float> rand_u_vel(-1.0,1.0);
std::uniform_real_distribution<float> rand_u_ang_vel(-2.0,2.0);

std::uniform_real_distribution<double> rand_xdot(0.0,1.0);
std::uniform_real_distribution<double> rand_x(0.0,5.0);
std::uniform_real_distribution<double> rand_theta(0.0,2.0*PI);
std::uniform_real_distribution<double> rand_thetadot(0.0,1);


std::uniform_real_distribution<double> dist_prob(0.0,1);


class dynamic_system 
{
public:  
  Eigen::Matrix4f A_dt = Eigen::Matrix4f::Zero(); //Discrete time A_matrix(dt = 0.01) (From MATLAB)
  Eigen::MatrixXf B_dt = Eigen::MatrixXf::Zero(4,2); //Discrete time B_matrix(dt=0.01) (From MATLAB)
  Eigen::Matrix4f C_dt = Eigen::Matrix4f::Zero(); //Discrete time A_matrix(dt = 0.01) (From MATLAB)
  Eigen::MatrixXf D_dt = Eigen::MatrixXf::Zero(4,2); //Discrete time B_matrix(dt=0.01) (From MATLAB)
  dynamic_system();
};

dynamic_system::dynamic_system():A_dt(4,4), B_dt(4,2)
{
  A_dt << 1.0 ,0.007592 ,0.0 ,0.0 
          ,0.0 ,0.5606 ,0.0 ,0.0
          ,0.0 ,0.0 ,1.0 ,0.001705
          ,0.0 ,0.0 ,0.0 ,0.002881;
  B_dt << 0.001579,0.0, 
           0.2882,0.0,
          0.0 ,0.008201,
          0.0 , 0.9858;
};

int sec2msec(double sec)
{
  return sec*100;
}

Eigen::Vector4f propagate(Eigen::Vector4f x_k,Eigen::Vector2f u_k, dynamic_system din_sytem,double prop_time)
{
  for(int i = 0; i < sec2msec(prop_time) ; ++i)
  {
    // printf("Iteration %d \n",i);
    x_k = din_sytem.A_dt*x_k + din_sytem.B_dt*u_k;
    // std::cout << x_k << std::endl;
  }
  return x_k;
};

Eigen::Vector4f random_state()
{
  Eigen::Vector4f x_rand; 
  x_rand(0) = rand_x(generator);
  x_rand(1) = rand_xdot(generator);
  x_rand(3) = rand_theta(generator);
  x_rand(4) = rand_thetadot(generator);
  return x_rand;
}

float error(Eigen::Vector4f x_goal, Eigen::Vector4f x_near)
{
  float error = 0; 
  for(int i = 0; i < x_goal.size(); ++i)
  {
    error+= pow(x_goal(i)-x_near(i),2);
  }
  return error;
}

int main()
{

  int count = 0;

  double prop_time = 0; //1 second of propagation

  float u_vel = rand_u_vel(generator); // m/s
  float u_ang_vel = rand_u_ang_vel(generator); // rad/s
  float error_dist=0;

  // float x_0 = 0.0; //m Initial forward displacement;
  // float ang_0 = 0.0; //rad Initial angular displacement;

  bool reached = false;

  Eigen::Vector4f x_0(0.915923,0.114889,-0.23683,-0.0296466);
  Eigen::Vector4f x_prop(0,0,0,0);
  Eigen::Vector4f x_goal(1,0,-0.3,0);
  Eigen::Vector2f u_k(u_vel,u_ang_vel);

  dynamic_system Husky;

  printf("T_prop: %.2f seconds; u_vel : %.6f m/s ; u_ang_vel : %.6f rad/s \n",prop_time,u_k(0),u_k(1));

  while(reached != true)
  {
    count +=1;
    u_k(0) = rand_u_vel(generator);
    u_k(1) = rand_u_ang_vel(generator);
    prop_time = rand_t_prop(generator);
    x_prop = propagate(x_0,u_k,Husky,prop_time);
    error_dist = error(x_goal,x_prop);
    printf("T_prop: %.2f seconds; u_vel : %.6f m/s ; u_ang_vel : %.6f rad/s ; error : %.6f \n",prop_time,u_k(0),u_k(1),error_dist);
    if(error_dist < 0.01)
    {
      printf("Goal Reached in count # %d \n",count);
      printf("T_prop: %.2f seconds; u_vel : %.6f m/s ; u_ang_vel : %.6f rad/s ; error : %.6f \n",prop_time,u_k(0),u_k(1),error_dist);
      reached = true;
    }
    if(count > 100000)
    {
      printf("Count excedeed\n");
      reached = true;
    }
  };

  std::cout << "Initial Condition" << std::endl;
  std::cout << x_0 << std:: endl;
  // x_k = propagate(x_k,u_k,Husky,prop_time);
  std::cout << "After " << sec2msec(prop_time) << " iterations" << std::endl;
  std::cout << x_prop << std::endl;
  std::cout << "Goal Condition" << std::endl;
  std::cout << x_goal << std::endl;

  // std::cout<< "A_dt"<< std::endl;
  // std::cout << A_dt << std::endl;
  // std::cout<< "B_dt"<< std::endl;
  // std::cout << B_dt << std::endl;
  // std::cout<<"One iteration" << std::endl;
  // std::cout<< x_k << std::endl;

}


