#ifndef DYNAMICS    
    #define DYNAMICS
    #include <math.h>
    #include <algorithm>
    #include <chrono>
    #include <random>
    #include <stdexcept>
    #include <regex> // For regex and split logic
    #include <iostream> // cout, endl
    #include <fstream> // For reading/writing files
    #include "state_template.h"
    #include "dynamics.h"

    #define PI 3.141592654
    
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();

    std::default_random_engine generator(seed);
    std::uniform_real_distribution<double> rand_t_prop(0.0,8.0);
    
    std::uniform_real_distribution<float> rand_u_vel(-1.0,1.0);
    std::uniform_real_distribution<float> rand_u_ang_vel(-2.0,2.0);

    std::uniform_real_distribution<double> rand_xdot(0.0,1.0);
    std::uniform_real_distribution<double> rand_x(0.0,5.0);
    std::uniform_real_distribution<double> rand_theta(0.0,2.0*PI);
    std::uniform_real_distribution<double> rand_thetadot(0.0,1);

    std::uniform_real_distribution<double> dist_prob(0.0,1);

    Xstate random_state()
    {
        Xstate x_random(rand_x(generator),rand_xdot(generator),rand_theta(generator),rand_thetadot(generator));
        return x_random;
    }

    double error(Xstate& x_goal,Xstate& x_near)
    {
        // auto x_goal = x_sgoal.getPointer();
        // auto x_near = x_snear.getPointer();
        double error = 0; 
        for(int i = 0; i < x_goal.size(); ++i)
        {
            error+= pow(x_goal[i]-x_near[i],2);
        }
        return sqrt(error);
    }

    int main()
    {

        int count = 0;

        double prop_time = 0; //1 second of propagation

        double u_vel = rand_u_vel(generator); // m/s
        double u_ang_vel = rand_u_ang_vel(generator); // rad/s
        double error_dist=0;

        bool reached = false;

        Xstate x_0(0.69,0.23,-0.5,-0.04);
        Xstate x_prop(0,0,0,0);
        Xstate x_goal(1,0,-0.3,0);
        Ustate u_k(1,1);
        // printf("X prop before copy \n");
        // std::cout << x_prop << std::endl;
        // printf("Testing copy operator\n");
        // x_prop = x_goal;
        // std::cout<< x_prop << std::endl;
        printf("T_prop: %.2f seconds; u_vel : %.6f m/s ; u_ang_vel : %.6f rad/s \n",prop_time,u_k[0],u_k[1]);

        while(reached != true)
        {
            count +=1;
            u_k[0] = rand_u_vel(generator);
            u_k[1] = rand_u_ang_vel(generator);
            u_k.set_tprop(rand_t_prop(generator));
            x_prop.propagate(x_0,u_k);
            error_dist = error(x_goal,x_prop);
            printf("T_prop: %.2f seconds; u_vel : %.6f m/s ; u_ang_vel : %.6f rad/s ; error : %.6f \n",u_k.get_tprop(),u_k[0],u_k[1],error_dist);
            // printf("T_prop: %.2f seconds; u_vel : %.6f m/s ; u_ang_vel : %.6f rad/s ; error : %.6f \n",prop_time,u_k[0],u_k[1],error_dist);
            if(error_dist < 0.15)
            {
            printf("Goal Reached in count # %d \n",count);
            printf("T_prop: %.2f seconds; u_vel : %.6f m/s ; u_ang_vel : %.6f rad/s ; error : %.6f \n",u_k.get_tprop(),u_k[0],u_k[1],error_dist);
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
        std::cout << "After " << sec2msec(u_k.get_tprop())<< " iterations" << std::endl;
        std::cout << x_prop << std::endl;
        std::cout << "Goal Condition" << std::endl;
        std::cout << x_goal << std::endl;
    }

#endif 