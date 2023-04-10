/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
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
#include <KRRT.h>
#include <fssimplewindow.h>
/* Input Arguments */

#define PI 3.141592654
/** Your final solution will be graded by an grading script which will
 * send the default 6 arguments:
 *    map, numOfDOFs, commaSeparatedStartPos, commaSeparatedGoalPos, 
 *    whichPlanner, outputFilePath
 * An example run after compiling and getting the planner.out executable
 * >> ./planner.out map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 0 output.txt
 * See the hw handout for full information.
 * If you modify this for testing (e.g. to try out different hyper-parameters),
 * make sure it can run with the original 6 commands.
 * Programs that do not will automatically get a 0.
 * */

// int main(int argc, char ** argv) 
// {
// 	KRRT RRT;
// 	RRT.LoadMap(argv[1]);

// 	if(RRT.one_shot_plan())
// 	{
// 		RRT.set_objects();

// 		FsOpenWindow(0,0,1000,1000,1);

//         std::cout << "Rendering planner" << std::endl;

//         // FsOpenWindow(0,0,w_width,w_height,1);
//         for(;;)
//         {
//             FsPollDevice();
//             if(FSKEY_ESC==FsInkey())
//             {
//                 break;
//             }

// 			RRT.ResetPos();
			
//             u_k = plan[idx]->getUstate();

//             double prop_time = u_k.get_tprop();
//             // // Xstate x_prop(plan[idx]->getXstate());
//             // x_planned = plan[idx]->getXstate();
//             Xstate x_prop;
//             for(int i = 0; i < sec2msec(prop_time) ; ++i)
//             {
//                 x_prop[0] = x_k[0] + u_k[0]*cos(x_k[2])*h;
//                 x_prop[1] = x_k[1] + u_k[0]*sin(x_k[2])*h;
//                 x_prop[2] = x_k[2] + u_k[1]*h;
//                 x_prop[3] = 0;

//                 x_k = x_prop;

//                 //Rendering

//                 // updte_pos_obj(x_k);

//                 husky_robot.Move(x_prop[0],x_prop[1],x_prop[2]);
//                 path.Move(x_prop[0],x_prop[1],x_prop[2]);
//                 t.Move(x_prop[0],x_prop[1],x_prop[2]);

//                 // draw_obj();

//                 map_1.renderMap();

//                 start_pos.Draw_object();
//                 start_pos.Draw_coords();
                
//                 goal_pos.Draw_object();
//                 goal_pos.Draw_coords();

//                 husky_robot.Draw_object_Angle();

//                 path.Draw_Path();
//                 t.Draw_tether();
                
//                 glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
                
//                 FsSwapBuffers();
//                 // FsSleep(1);
//             }
//             ++idx; 
//         }
// 	}
// 	return 0;
// }
