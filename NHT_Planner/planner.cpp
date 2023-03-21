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
#include <node.h>
#include <fssimplewindow.h>
#include <map.h>
#include <ysglfontdata.h>
/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define GETMAPINDEX_LL0(X, Y, XSIZE, YSIZE) ((YSIZE-Y-1)*XSIZE + X)
#define GETMAPINDEX_LL1(X, Y, XSIZE, YSIZE) ((YSIZE-Y)*XSIZE + X)
#define GETMAPINDEX_UL0(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)
#define GETMAPINDEX_UL1(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654
#define DEBUG false
//the length of each link in the arm
#define LINKLENGTH_CELLS 10


auto seed = std::chrono::system_clock::now().time_since_epoch().count();
//1234 12345 123456 1234567 12345678
std::default_random_engine gen(seed);

std::uniform_real_distribution<double> rand_t_prop(0.0,5.0);

std::uniform_int_distribution<int> rand_u_vel(0,100);
std::uniform_real_distribution<double> rand_u_ang_vel(-2.0,2.0);

std::uniform_real_distribution<double> rand_xdot(-1.0,1.0);
std::uniform_real_distribution<double> rand_x(0,70.0);
std::uniform_real_distribution<double> rand_theta(-PI,PI);
std::uniform_real_distribution<double> rand_thetadot(-1.0,1.0);

// std::uniform_real_distribution<double> rand_map_coordsx(0,100);
// std::uniform_real_distribution<double> rand_map_coordsy(0,100);

std::uniform_int_distribution<int> rand_map_coordsx(0,999);
std::uniform_int_distribution<int> rand_map_coordsy(0,999);

std::uniform_real_distribution<double> dist_prob(0.0,1);


/// @brief 
/// @param filepath 
/// @return map, x_size, y_size


// #include "Vertice.h"

//Added by AF

class object 
{
	public:
		GLfloat x=0,y=0,angle=0,w=0,h=0;
		GLubyte r=0,g=0,b=0;
		int coords[2] = {0,0};
		object(void);
		object(GLfloat i_x, GLfloat i_y, GLfloat i_w, GLfloat i_h);
		void setColor(GLubyte i_r, GLubyte i_g, GLubyte i_b);
		void setDim(GLfloat i_w, GLfloat i_h);
		void Move(GLfloat i_x, GLfloat i_y, GLfloat i_angle);
		void Draw_object(void);
		void Draw_in_center(void);
		void Draw_object_Angle(void);
		void Draw_coords(void);
};

object::object(){};
object::object(GLfloat i_x, GLfloat i_y, GLfloat i_w, GLfloat i_h):
x{i_x},y{i_y},w{i_w},h{i_h}{};

void object::setColor(GLubyte i_r, GLubyte i_g, GLubyte i_b)
{
	this->r = i_r;
	this->g = i_g;
	this->b = i_b;
}
void object::setDim(GLfloat i_w, GLfloat i_h)
{
	this->h = i_h;
	this->w = i_w;
}

void object::Move(GLfloat i_x, GLfloat i_y, GLfloat i_angle)
{
	this->x = i_x;
	this->y = i_y;
	this->angle = i_angle; 
}

void object::Draw_object()
{
	glColor3ub(this->r,this->g,this->b);

	glBegin(GL_QUADS);

	glTexCoord3f(0.0,0.0,0.0);
	glVertex3f(x,y,0); //0,0

	glTexCoord3f(1.0,0.0,0.0);
	glVertex3f(x+w,y,0); //1,0

	glTexCoord3f(1.0,1.0,0);
	glVertex3f(x+w,y+h,0); // 1,1

	glTexCoord3f(0.0,1.0,0.0);
 	glVertex3f(x,y+h,0); // 0,1

	glEnd(); 

}

void object::Draw_in_center()
{
	glColor3ub(this->r,this->g,this->b);

	glBegin(GL_QUADS);

	glTexCoord3f(0.0,0.0,0.0);
	glVertex3f(x-w/2 ,y-h/2,0); //0,0

	glTexCoord3f(1.0,0.0,0.0);
	glVertex3f(x+w-w/2,y-h/2,0); //1,0

	glTexCoord3f(1.0,1.0,0);
	glVertex3f(x+w-w/2,y+h-h/2,0); // 1,1

	glTexCoord3f(0.0,1.0,0.0);
 	glVertex3f(x-w/2,y+h-h/2,0); // 0,1

	glEnd(); 
}

void object::Draw_object_Angle()
{
	
	if(this->angle < 0)
	{
		this->angle = 2*PI-this->angle;
	}
	this->angle = (this->angle/PI) * 180;

	// glLoadIdentity();
	glPushMatrix();

	glTranslatef(x+w/2,y+h/2, 0);      
	glRotatef(angle, 0.0f, 0.0f, -1.0f);
	glTranslatef(-x-w/2, -y-h/2, 0);

	glColor3ub(this->r,this->g,this->b);

	glBegin(GL_QUADS);

	glTexCoord3f(0.0,0.0,0.0);
	glVertex3f(x ,y,0); //0,0

	glTexCoord3f(1.0,0.0,0.0);
	glVertex3f(x+w,y,0); //1,0

	glTexCoord3f(1.0,1.0,0);
	glVertex3f(x+w,y+h,0); // 1,1

	glTexCoord3f(0.0,1.0,0.0);
 	glVertex3f(x,y+h,0); // 0,1
	glEnd(); 

	
	glColor3ub(0,0,0);

	glBegin(GL_LINES);
	glVertex2i(x+w/2,y+h/2);
	glVertex2i(x+w/2+20,y+h/2);
	glEnd();



	glPopMatrix();
}

void object::Draw_coords()
{
	glColor3ub(0,0,0);
	glRasterPos2i(x+w,y+2*h);
	char str[256];
	int i_x = x;
	int i_y = y;
	sprintf(str,"%d,%d",coords[0],coords[1]);
	YsGlDrawFontBitmap12x16(str);
}

double euclidean(Xstate& x_goal,Xstate& x_near)
{
	// auto x_goal = x_sgoal.getPointer();
	// auto x_near = x_snear.getPointer();
	double dist = 0; 
	for(int i = 0; i < x_goal.size(); i+=1)
	{
		dist+= pow(x_goal[i]-x_near[i],2);
	}
	return sqrt(dist);
}

double euclidean(double xf,double yf,double xi,double yi)
{
	return sqrt(pow(xf-xi,2)+pow(yf-yi,2));
}

double euclidean(double *coord_f,double *coord_b)
{
	return sqrt(pow(coord_f[0]-coord_b[0],2)+pow(coord_f[1]-coord_b[0],2));
}



// double euclidean(Xstate& x_goal,Xstate& x_near)
// {
// 	// auto x_goal = x_sgoal.getPointer();
// 	// auto x_near = x_snear.getPointer();
// 	double dist = 0; 
// 	for(int i = 0; i < x_goal.size(); ++i)
// 	{
// 		dist+= pow(x_goal[i]-x_near[i],2);
// 	}
// 	return sqrt(dist);
// }

double calc_radius()
{
	double d = 4; 
	double gamma = 20;
	double delta = (PI*PI)/2;
	double V = 2;
	return (gamma/delta)*pow((log(V)/V),1/d);
}

int nearest_n_idx(Xstate x_rand,std::vector<node*>& tree)
{
	double min = std::numeric_limits<double>::infinity();
	// double r = calc_radius();
	double r = 9;
	int min_node_idx = -1;
	if(!tree.empty())
	{
		for(int i = 0; i < tree.size(); ++i)
		{ 
			double dist = euclidean(x_rand,tree[i]->getXstate());
				if(dist < r)
				{
					if(dist < min)
					{
						min_node_idx = i;
						min = dist;	
					}
				}
		}
	}
	return min_node_idx;

}

int nearest_n_idx_map(Xstate x_rand,std::vector<node*>& tree)
{
	double min = std::numeric_limits<double>::infinity();
	// double r = calc_radius();
	double r = 20;
	int min_node_idx = -1;
	if(!tree.empty())
	{
		for(int i = 0; i < tree.size(); ++i)
		{ 
			Xstate x_t(tree[i]->getXstate());
			double dist = euclidean(x_rand.map_coords,x_t.map_coords);
				if(dist < r)
				{
					if(dist < min)
					{
						min_node_idx = i;
						min = dist;	
					}
				}
		}
	}
	return min_node_idx;

}

void CleanUp(std::vector<node*>& tree)
{
	for(node* p:tree)
	{
		delete p;
	}
	tree.clear();
}

void getPlan(std::vector<node*>& plan,std::vector<node*>& tree)
{
	for(node* p = tree.back(); p != nullptr; p = plan.back()->getParent())
	{
		plan.push_back(p);
	}
	reverse(plan.begin(),plan.end());
	};

bool small_uvel(double num)
{
	if(num > -0.1 && num < 0.1 )
		return true;
	else
		return false;
}

bool small_uangvel(double num)
{
	if(num > -0.17 && num < 0.17 ) // 
		return true;
	else
		return false;
}

bool small_control(Ustate& u)
{
	if(u[0] > -0.1 && u[0] < 0.1 )
		if(u[1] > -0.17 && u[1] < 0.17)
			if(u.get_tprop() > 2)
				return true;
			else
				return false;
		else
			return false;
	else
		return false;

}

void polar2coord(double* coords,Xstate x)
{
	coords[0] = x[0]*cos(x[2]);
	coords[1] = x[0]*sin(x[2]);
}

void meter2pixel(double* coords_meters)
{
	coords_meters[0] = std::round(coords_meters[0]/0.01);
	coords_meters[1] = std::round(coords_meters[1]/0.01);
}
void print_pos(Xstate& x)
{
	double coords[2]={0,0};
	polar2coord(coords,x); 
	std::cout<< std::round(coords[0]/0.01) << "," << std::round(coords[1]/0.01) << std::endl;
}

void polar2meter(double* coords,double x,double theta)
{
	coords[0] = std::round((x*cos(theta))/0.01);
	coords[1] = std::round((x*sin(theta))/0.01);
}

double calc_angle(double xf,double yf,double xi,double yi)
{	
	double angle = atan2(yf-yi,xf-xi);
	if(angle < 0)
	{
		angle = 2*PI+angle;
	}
	return angle;

}
double calc_angle(double* xf,double* xi)
{	
	double angle = atan2(xf[1]-xi[1],xf[0]-xi[0]);
	if(angle < 0)
	{
		angle = 2*PI+angle;
	}
	return angle;
}

void map2block(double *map_coords,map map_1)
{
	map_coords[0] = map_1.block_x*std::round(map_coords[0]);
	map_coords[1] = map_1.block_y*(map_1.height - std::round(map_coords[1]));
}

//-------------------------Below Added by AF----------------------------------------------------
//-------------------------BELOW RRT Functions & Code ------------------------------------------------
static void planner(map& map_1,
			Xstate& x_0, 
			Ustate& u_0,
			Xstate& x_goal,
			std::vector<node*>& plan,
			std::vector<node*>& tree)
{
	int K = 100000;
	Xstate x_rand;
	Xstate x_prop;
	Ustate u_k;
	bool reached = false;
	double prop_time=0;

	tree.push_back(new node(0,euclidean(x_goal,x_0),nullptr,u_0,x_0));
	std::cout<< "Number of samples: "<< K << std::endl;   
	for(int i = 0; i < K; ++i)
	{
		// std::cout<< "Number of samples: "<< i << std::endl; 
		double prob = dist_prob(gen); //Creating random number representing probability 
		if(prob > 0.75) //Goal biasing by 5% 
		{
			x_rand = x_goal; //Assigning qrandom to be goal
		}
		else
		{
			// Xstate x_r(rand_x(gen),rand_xdot(gen),rand_theta(gen),rand_thetadot(gen));
			// x_rand = x_r;
			auto seed = std::chrono::system_clock::now().time_since_epoch().count();
			std::default_random_engine gen(seed);
			// srand(time(NULL));
			// Xstate x_r(rand()%50,rand()%50,rand_theta(gen),0);
			Xstate x_r((double)rand_map_coordsx(gen)/10.0,(double)rand_map_coordsy(gen)/10.0,rand_theta(gen),0);
			x_rand = x_r;
		}

		int nn_idx = nearest_n_idx(x_rand,tree);//Loops through the entire list for the closest neighbor

		if(nn_idx >= 0)
		{
			Xstate x_near = tree[nn_idx]->getXstate(); //Grabs that qnear
			Xstate x_min;
			Ustate u_min;
			int count = 0;
			double min_dist = std::numeric_limits<double>::infinity(); 			
			for(int i = 0; i < 20000; ++i)
			{
				u_k[0] = (double) rand_u_vel(gen)/100.0;
				u_k[1] = rand_u_ang_vel(gen);
				u_k.set_tprop(rand_t_prop(gen));
 
				x_prop = propagate(x_near,u_k,map_1.map_ptr,map_1.width,map_1.height);
				// x_prop.propagate(x_near,u_k);

				double dist = euclidean(x_rand,x_prop);
				if(dist < 0.1)
				{	
					u_min = u_k;
					min_dist = dist;
					x_min = x_prop;
					break;
					// printf("T_prop: %.2f seconds; u_vel : %.6f m/s ; u_ang_vel : %.6f rad/s ; error : %.6f \n",u_k.get_tprop(),u_k[0],u_k[1],min_dist);
				}
				else if(dist < min_dist)
				{
					u_min = u_k;
					min_dist = dist;
					x_min = x_prop;
				}
			};
			// std::cout << x_min.map_coords[0] << ", " << x_min.map_coords[1] << std::endl;
			// std::cout << x_min[0] << std::endl;
			// printf("Xrand_lin_x: %f ; Xnear_lin_x: %f ; Xprop_min_lin_x:%f \n",x_rand[0],x_near[0],x_min[0]);
			if(euclidean(x_min,tree.back()->getXstate()) > 0.001) //Avoid repetition of node
			{
				tree.push_back(new node(1,euclidean(x_goal,x_min),tree[nn_idx],u_min,x_min)) ;
				double dist2goal = euclidean(x_goal,x_min);
			// double dist2goal = euclidean(x_goal,tree.back()->getXstate());
				if(dist2goal < 1)
				{
					printf("Goal found at K: %d\n", i);
					printf("T_prop: %.2f seconds; u_vel : %.6f m/s ; u_ang_vel : %.6f rad/s ; error : %.6f \n",u_min.get_tprop(),u_min[0],u_min[1],min_dist);
					printf("Initial State: \n");
					std::cout << x_0 << std::endl;
					printf("Final state: \n");
					std::cout << x_min << std::endl;
					printf("Goal state:\n");
					std::cout<<x_goal<< std::endl;
					getPlan(plan,tree);
					// CleanUp(tree);
					return;
				}
			}
		}
	}

	printf("No plan found \n");
	// CleanUp(tree);
	//no plan by default
    return;
}


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
int main(int argc, char ** argv) 
{
	double* map_t = nullptr;
	int coords[2] ={0,0};
	double coords_start[2]={30,15}; //20,25 ; 30,15 (x,y)
	double coords_goal[2]={7,46}; // 7, 46 (x,y)

	int x_size=0, y_size=0;

	map map_1; 
	std::vector<node*> plan; 
	std::vector<node*> tree;

	Ustate u_start(0,0);
	Xstate x_prop;
	Xstate x_start(coords_start[0],coords_start[1],PI/2,0);
	Xstate x_goal(coords_goal[0],coords_goal[1],PI/2,0);


	object husky_robot;
	object start_pos;
 	object goal_pos;

	map_1.loadMap(argv[1]);
	int start_idx = GETMAPINDEX_LL0(coords_start[0],coords_start[1],map_1.width,map_1.height);
	int goal_idx = GETMAPINDEX_LL0(coords_goal[0],coords_goal[1],map_1.width,map_1.height);

	if(map_1.map_ptr[start_idx] == 1)
	{
		printf("Invalid start location\n");
		return 0;
	}
	else if(map_1.map_ptr[goal_idx] == 1)
	{
		printf("Invalid Goal location \n");
		return 0;
	}

	printf("Initial condition in X,Y:\n");
	print_pos(x_start);
	printf("Goal condition in X,Y: \n");
	print_pos(x_goal);
	u_start.set_tprop(0);
	printf("radius to test near neighbors:\n");
	std::cout << calc_radius() << std::endl;

	
	// polar2meter(coords_start,x_start[0],x_start[2]);
	x_start.map_coords[0] = coords_start[0];
	x_start.map_coords[1] = coords_start[1];
	// polar2meter(coords_goal,x_goal[0],x_goal[2]);
	x_goal.map_coords[0] = coords_goal[0];
	x_goal.map_coords[1] = coords_goal[1];

	std::cout << x_start.map_coords[1] << std::endl;
	// for(int i = 0; i < map_1.width*map_1.height ; ++i)
	// {
	// 	if(i%50 == 0)
	// 	{
	// 		std::cout << std::endl;
	// 	}
	// 	std::cout << map_1.map_ptr[i] << " ";

	// }
	{
		Ustate u_test(1,0);
		u_test.set_tprop(1);
		Xstate x_prop_test;
		Xstate x_prop_test2;
		Xstate x_prop_test3;
		x_prop_test.propagate(x_start,u_test);

		printf("Test:\n");
		std::cout << x_prop_test << std::endl;
		printf("Test 2:\n");
		x_prop_test2 = propagate(x_start,u_test,map_1.map_ptr,map_1.width,map_1.height);
		std::cout << x_prop_test2 << std::endl;
		printf("Test 3:\n");
		x_prop_test3 = propagate(x_prop_test2,u_test,map_1.map_ptr,map_1.width,map_1.height);
		std::cout << x_prop_test3 << std::endl;
		printf("Test 4: \n");
		x_prop_test3 = propagate(x_prop_test3,u_test,map_1.map_ptr,map_1.width,map_1.height);
		std::cout << x_prop_test3 << std::endl; 
	}
	auto start_time = std::chrono::system_clock::now();
	planner(map_1,x_start,u_start,x_goal,plan,tree);
	auto end_time = std::chrono::system_clock::now();
	auto time_delay = std::chrono::duration_cast<std::chrono::nanoseconds> (end_time-start_time);
	float time_passed = time_delay.count()*1e-9;
	
	std::cout<<"--------------- RESULTS---------------"<<std::endl;
	std::cout<<"Planning time: "<< time_passed << " seconds" << std::endl;
	
	int plan_size = plan.size();

	start_pos.setDim(20,20);
	start_pos.setColor(255,0,0);
	start_pos.coords[0] = coords_start[0];
	start_pos.coords[1] = coords_start[1];
	start_pos.Move(map_1.block_x*coords_start[0],map_1.block_y*(map_1.height-coords_start[1]),0);

	goal_pos.setDim(20,20);
	goal_pos.setColor(255,0,0);
	goal_pos.coords[0] = coords_goal[0];
	goal_pos.coords[1] = coords_goal[1];
	goal_pos.Move(map_1.block_x*coords_goal[0],map_1.block_y*(map_1.height-coords_goal[1]),0);

	husky_robot.setDim(20,15);
	husky_robot.setColor(255,240,10);
	husky_robot.Move(map_1.block_x*coords_start[0],map_1.block_y*(map_1.height-coords_start[1]),0);

	int idx = 0;
	int w_width = 750;
	int w_height = 800;

	Xstate x_k(plan[0]->getXstate());
	double h = 0.01;
	FsOpenWindow(0,0,w_width,w_height,1);


		for(;;)
		{
			FsPollDevice();
			if(FSKEY_ESC==FsInkey())
			{
				break;
			}
			if(idx >= plan_size)
 			{	
				idx = 0;
				x_k = plan[0]->getXstate(); 
				husky_robot.Move(map_1.block_x*coords_start[0],map_1.block_y*(map_1.height-coords_start[1]),0);
			}	 
			Ustate u_k(plan[idx]->getUstate());
			// Xstate x_prop(plan[idx]->getXstate());
			int t_prop = sec2msec(u_k.get_tprop());

			// husky_robot.Move(map_1.block_x*std::round(x_prop[0]),map_1.block_y*(map_1.height-std::round(x_prop[1])),x_prop[2]);

			// // //Rendering

			// glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
			// // glLoadIdentity();

			// map_1.renderMap();

			// start_pos.Draw_object();
			// start_pos.Draw_coords();
			
			// goal_pos.Draw_object();
			// goal_pos.Draw_coords();

			// husky_robot.Draw_object_Angle();

			// FsSwapBuffers();
			// FsSleep(250);

			for(int i = 0; i < t_prop ; ++i)
			{

				if(FSKEY_ESC==FsInkey())
				{
					break;
				}
				x_prop[0] = x_k[0] + u_k[0]*cos(x_k[2])*h;
				x_prop[1] = x_k[1] + u_k[0]*sin(x_k[2])*h;
				x_prop[2] = x_k[2] + u_k[1]*h;
				x_prop[3] = 0;

				x_k = x_prop;

				double coords[2] ={0,0};
				// map2block(coords,map_1);
				husky_robot.Move(map_1.block_x*x_prop[0],map_1.block_y*(map_1.height-x_prop[1]),x_prop[2]);

				//Rendering
				glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

				glLoadIdentity();
				map_1.renderMap();

				start_pos.Draw_object();
				start_pos.Draw_coords();
				
				goal_pos.Draw_object();
				goal_pos.Draw_coords();

				husky_robot.Draw_object_Angle();

				printf("%f,%f \n",x_prop[0],x_prop[1]);

				FsSwapBuffers();
				// FsSleep(1);
			}
			printf("Step: %d",idx);
			++idx; 
		}

	CleanUp(tree);
	plan.clear();
	return 0;
}
