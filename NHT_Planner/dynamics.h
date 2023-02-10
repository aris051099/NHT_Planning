#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>
#include <chrono>
#include <random>
#include <tuple>
#include <string>
#include <stdexcept>
#include <regex> // For regex and split logic
#include <iostream> // cout, endl
#include <fstream> // For reading/writing files
#include <assert.h>
#include <limits>
#include <queue>
#include <unordered_set>
#include <eigen3/Eigen/Dense>

#define PI 3.141592654
 
unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
//1234 12345 123456 1234567 12345678
std::default_random_engine generator(seed);
std::uniform_real_distribution<double> rand_t_prop(0.0,5);
std::uniform_real_distribution<float> rand_u_vel(0.0,1.0);
std::uniform_real_distribution<float> rand_u_ang_vel(0.0,2.0);

std::uniform_real_distribution<double> rand_xdot(0.0,1.0);
std::uniform_real_distribution<double> rand_x(0.0,5.0);
std::uniform_real_distribution<double> rand_theta(0.0,2.0*PI);
std::uniform_real_distribution<double> rand_thetadot(0.0,1);


std::uniform_real_distribution<double> dist_prob(0.0,1);

int sec2msec(double sec)
{
  return sec*100;
}

int main()
{

  double prop_time = rand_t_prop(generator); //1 second of propagation 
  float u_vel = rand_u_vel(generator); // m/s
  float u_ang_vel = rand_u_ang_vel(generator); // rad/s

  float x_0 = 0.0; //m Initial forward displacement;
  float ang_0 = 0.0; //rad Initial angular displacement;

  Eigen::Matrix4f A_dt; //Discrete time A_matrix(dt = 0.01) (From MATLAB)
  A_dt << 1.0 ,0.007592 ,0.0 ,0.0 
          ,0.0 ,0.5606 ,0.0 ,0.0
          ,0.0 ,0.0 ,1.0 ,0.001705
          ,0.0 ,0.0 ,0.0 ,0.002881;
  
  Eigen::MatrixXf B_dt(4,2); //Discrete time B_matrix(dt=0.01) (From MATLAB)
  B_dt << 0.001579,0.0, 
           0.2882,0.0,
          0.0 ,0.008201,
          0.0 , 0.9858;
  

  Eigen::Vector4f x_k(x_0,0.0,ang_0,0.0);
  Eigen::Vector2f u_k(1,u_ang_vel);

  printf("T_prop: %.2f seconds; u_vel : %.6f m/s ; u_ang_vel : %.6f rad/s \n",prop_time,u_k(0),u_k(1));

  std::cout << "Initial Condition" << std::endl;
  std::cout << x_k << std:: endl;

  for(int i = 1; i < sec2msec(prop_time) ; ++i)
  {
    // printf("Iteration %d \n",i);
    x_k = A_dt*x_k + B_dt*u_k;
    // std::cout << x_k << std::endl;
  }

  std::cout << "After " << sec2msec(prop_time) << " iterations" << std::endl;
  std::cout << x_k << std::endl;
  // std::cout<< "A_dt"<< std::endl;
  // std::cout << A_dt << std::endl;
  // std::cout<< "B_dt"<< std::endl;
  // std::cout << B_dt << std::endl;
  // std::cout<<"One iteration" << std::endl;
  // std::cout<< x_k << std::endl;

}


