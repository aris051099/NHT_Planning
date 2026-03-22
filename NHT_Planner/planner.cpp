/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <iostream>
#include <fstream>
#include <KRRT.h>
#include <fssimplewindow.h>


// Xstate propagate_one_step(const KRRT& RRT)
// {
// 	Xstate x_prop;
// 	Xstate x_k(RRT.x_p);
// 	Ustate u_k(RRT.u_k);
// 	double h = RRT.h;
//     double eps = RRT.eps;
//     x_prop[0] = x_k[0] + u_k[0]*cos(x_k[2])*h;
//     x_prop[1] = x_k[1] + u_k[0]*sin(x_k[2])*h;
//     x_prop[2] = x_k[2] + u_k[1]*h;
//     x_prop[3] = x_k[3] + eps*u_k[0]*h*sin(x_k[3]) + u_k[1]*h;

// 	return x_prop;
// }

  // int sec2msec(double sec)
  // {
  // return (std::round(sec*100.0)/100.0)*100;
  // }
int main(int argc, char ** argv) 
{
    KRRT RRT;

    RRT.LoadMap(argv[1]);

    // ObstacleFinder obstacleFinder(RRT.map_1.map_ptr, RRT.map_1.width, RRT.map_1.height);
    // std::vector<std::vector<int>> obstacles = obstacleFinder.findObstacles();

    // // Print the indices of cells in each obstacle
    // int x = 0;
    // int y = 0;
    // for (int i = 0; i < obstacles.size(); ++i) 
    // {
    //     std::cout << "Obstacle " << i + 1 << ": ";
    //     for (int j = 0; j < obstacles[i].size(); ++j) 
    //     {
    //     get2DCoordinates(obstacles[i][j],RRT.map_1.height,x,y);
    //     std::cout << x << "," << y << " ";
    //     }
    //     std::cout << std::endl;
    // }

  {
    //Testing of integration methods
    Xstate x1_euler;
    Xstate x1_rk4;
    Xstate x_euler;
    Xstate x_rk4; 
    Ustate u(0.9,0.1);
    for(int i = 0; i < 100 ; ++i)
    {
          x_euler = RRT.propagate_one_step(x1_euler,u);
          x1_euler = x_euler;
          x_rk4 = RRT.rk4step(x1_rk4,u,0.01);
          x1_rk4 = x_rk4;
    } 
  //Now print the results
  std::cout << "Euler: " << x_euler[0] << " " << x_euler[1] << " " << x_euler[2] << " " << x_euler[3] << std::endl;
  std::cout << "Rk4:" << x_rk4[0] << " " << x_rk4[1] << " " << x_rk4[2] << " " << x_rk4[3] << std::endl;
  }

	if(RRT.one_shot_plan())
	{

        RRT.myfile.open("C:/Users/arisa/Desktop/Path_Planning/NHT_Planning/NHT_Planner/results.csv");
        std::cout << "Rendering planner" << std::endl;

        RRT.set_objects();

        FsOpenWindow(0,0,1000,1000,1);
        for(;;)
        {
          FsPollDevice();
            if(FSKEY_ESC==FsInkey())
            {
                break;
            }
          RRT.ResetPos();
          
          RRT.u_k = RRT.plan[RRT.idx]->getUstate();

          // if(RRT.idx != 0)
          // {
          //   RRT.x_p = RRT.plan[RRT.idx-1]->getXstate();
          // }

          double prop_time = RRT.u_k.get_tprop();

          Xstate x_prop;
          Xstate x_prop2;
          for(int i = 0; i < RRT.sec2msec(prop_time)  ; ++i)
          {
            // x_prop = RRT.propagate_one_step(RRT.x_p,RRT.u_k);
            x_prop = RRT.rk4step(RRT.x_p,RRT.u_k,RRT.h);

            RRT.myfile << x_prop[3]  << ",";
            RRT.myfile << RRT.u_k[0] << ",";
            RRT.myfile << RRT.u_k[1] << ","<< "\n";

            RRT.x_p = x_prop;

            //Rendering

            glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

            RRT.updte_pos_obj(x_prop);
                    
            RRT.draw_obj();

            FsSwapBuffers();
            // FsSleep(1);
          }
          ++RRT.idx; 
        }
        RRT.myfile.close();
	}
	return 0;
}
