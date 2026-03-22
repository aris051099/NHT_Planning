/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include <iostream>
#include <fstream>
#include <KRRT.h>
#include <fssimplewindow.h>

int main(int argc, char** argv)
{
    KRRT RRT;
    RRT.LoadMap(argv[1]);

    // Integration method comparison (Euler vs RK4)
    {
        Xstate x_euler, x_rk4;
        Ustate u(0.9, 0.1);
        for(int i = 0; i < 100; ++i)
        {
            x_euler = RRT.propagate_one_step(x_euler, u);
            x_rk4   = RRT.rk4step(x_rk4, u, RRT.dynamics.h);
        }
        std::cout << "Euler: " << x_euler[0] << " " << x_euler[1]
                  << " "       << x_euler[2] << " " << x_euler[3] << std::endl;
        std::cout << "RK4:   " << x_rk4[0]   << " " << x_rk4[1]
                  << " "       << x_rk4[2]   << " " << x_rk4[3]   << std::endl;
    }

    if(RRT.one_shot_plan())
    {
        RRT.myfile.open("results.csv");
        std::cout << "Rendering planner" << std::endl;

        RRT.set_objects();

        FsOpenWindow(0, 0, 1000, 1000, 1);
        for(;;)
        {
            FsPollDevice();
            if(FSKEY_ESC == FsInkey())
                break;

            RRT.ResetPos();

            RRT.u_k = RRT.plan[RRT.idx]->getUstate();

            double prop_time = RRT.u_k.get_tprop();
            Xstate x_prop;

            for(int i = 0; i < RRT.sec2msec(prop_time); ++i)
            {
                x_prop = RRT.rk4step(RRT.x_p, RRT.u_k, RRT.dynamics.h);

                RRT.myfile << x_prop[3]    << ","
                           << RRT.u_k[0]  << ","
                           << RRT.u_k[1]  << "\n";

                RRT.x_p = x_prop;

                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                RRT.updte_pos_obj(x_prop);
                RRT.draw_obj();
                FsSwapBuffers();
            }
            ++RRT.idx;
        }
        RRT.myfile.close();
    }
    return 0;
}
