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
#include <Xstate.h>
#include <node.h>
#include <fssimplewindow.h>
#include <map.h>
#include <ysglfontdata.h>
#include <object.h>
#include <KDtree.h>
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
#define DEBUG true
//the length of each link in the arm
#define LINKLENGTH_CELLS 10


auto seed = std::chrono::system_clock::now().time_since_epoch().count();
// auto seed = 12;
//1234 12345 123456 1234567 12345678
std::default_random_engine gen(seed);

std::uniform_real_distribution<double> rand_t_prop(0.0,8);

std::uniform_int_distribution<int> rand_u_vel(50,100);
std::uniform_real_distribution<double> rand_u_ang_vel(-0.25,0.25);

std::uniform_int_distribution<int> near_rand_u_vel(10,50);
std::uniform_real_distribution<double> near_rand_u_ang_vel(-2.0,2.0);

std::uniform_real_distribution<double> rand_xdot(-1.0,1.0);
std::uniform_real_distribution<double> rand_x(0,70.0);
std::uniform_real_distribution<double> rand_theta(-PI,PI);
std::uniform_real_distribution<double> rand_thetadot(-1.0,1.0);

std::uniform_int_distribution<int> rand_map_coordsx(0,999);
std::uniform_int_distribution<int> rand_map_coordsy(0,999);

std::uniform_real_distribution<double> dist_prob(0.0,1);



/// @brief 
/// @param filepath 
/// @return map, x_size, y_size


// #include "Vertice.h"

//Added by AF
double euclidean(Xstate& x_goal,const Xstate& x_near)
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
double L2_norm(Xstate& x_s)
{
	double sum = 0;
	for(int i = 0; i < x_s.size(); ++i )
	{
		sum+=pow(x_s[i],2);
	}
	return sqrt(sum);
}
double LQR_Cost(Xstate& x_k, Ustate& u_k)
{
	double t_prop = sec2msec(u_k.get_tprop());
	double sum = 0;
	for(int i = 0; i < t_prop ; ++i)
	{
		sum+= u_k[0]*1*u_k[0] + u_k[1]*5*u_k[1];
	}
	sum += u_k.get_tprop()*0.1*u_k.get_tprop();
 	return sum;
}
double calc_radius(std::vector<node*>& tree)
{
	double d = 3; 
	double gamma = 150;
	double delta = (PI*PI)/2;
	double V = tree.size();
	return (gamma/delta)*pow((log(V)/V),1/d);
}

int nearest_n_idx(Xstate x_rand,std::vector<node*>& tree)
{
	double min = std::numeric_limits<double>::infinity();
	// double r = 9;
	double r = L2_norm(x_rand);
	if(tree.size() > 1)
	{
		r = std::min(L2_norm(x_rand),calc_radius(tree));
	}
	int min_node_idx = -1;
	if(!tree.empty())
	{
		for(int i = 0; i < tree.size(); ++i)
		{ 
			double dist = euclidean(x_rand,tree[i]->getXstate());
				if(dist > 0 && dist < r)
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

void nearest_nn_idx(Xstate x_rand,double r,std::vector<node*>& tree,std::vector<int>& nn_idxs)
{
	nn_idxs.clear();
	double min = std::numeric_limits<double>::infinity();
	// double r = calc_radius();
	// double r = 9;
	// int min_node_idx = -1;
	if(!tree.empty())
	{
		for(int i = 0; i < tree.size(); ++i)
		{ 
			double dist = euclidean(x_rand,tree[i]->getXstate());
				if(dist > 0 && dist < r)
				{
					nn_idxs.push_back(i);
				}
		}
	}
}
void CleanUp(std::vector<node*>& tree, KDTree& Ktree)
{
	// for(node* p:tree)
	// {
	// 	delete p; 
	// 	p = nullptr;
	// }
	bool isRoot = false;
	for(size_t i =0;i < tree.size(); ++i)
	{
		if(tree[i] == Ktree.getRoot())
		{
			Ktree.removeRoot();
			isRoot = true;
		}
		if(!isRoot)
		{
			delete tree[i];
			tree[i] = nullptr;
		}
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
double calc_angle(int xf[],int xi[])
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

void steer(Xstate x_near,Xstate x_rand,map map_1,Xstate& x_best,Ustate& u_best, double prob,bool near_goal)
{
	// Xstate x_near = tree[nn_idx]->getXstate(); //Grabs that qnear
	Xstate x_prop;
	Ustate u_k;
	double min_dist = std::numeric_limits<double>::infinity(); 			
	for(int i = 0; i < 20; ++i)
	{
		u_k[0] = (double) rand_u_vel(gen)/100.0;
		u_k[1] = rand_u_ang_vel(gen);
		u_k.set_tprop(rand_t_prop(gen));
		if(near_goal && prob > 0.95)
		{
			u_k[0] = (double) near_rand_u_vel(gen)/100.0;
			u_k[1] = near_rand_u_ang_vel(gen);
			u_k.set_tprop(rand_t_prop(gen));
		}

		x_prop = propagate(x_near,u_k,map_1.map_ptr,map_1.width,map_1.height);

		double dist = euclidean(x_rand,x_prop);
		if(dist < min_dist)
		{
			// x_prop.state = 1; //Steered 
			u_best = u_k;
			min_dist = dist;
			x_best = x_prop;
		}
	};
}

// bool ObstacleFree(Xstate x_near,Xstate x_rand,map map_1,Xstate& x_best,Ustate& u_best)
// {
// 	steer(x_near,x_rand,map_1,x_best,u_best);
// 	if(x_best.state!=2)
// 	{
// 		return true;
// 	}
// 	else
// 	{
// 		return false;
// 	}
// }

void Add_Edge(node *q_min,node *q_near)
{
	q_min->childs.emplace_back(q_near);
	q_near->childs.emplace_back(q_min);
}

void Update_parent(node* q_new,node* q_near)
{
	for(auto q_n:q_near->childs)
	{
		if(q_n->getParent() == q_near)
		{
			q_n->setParent(q_new);
		}
	}
}


//-------------------------Below Added by AF----------------------------------------------------
//-------------------------BELOW RRT Functions & Code ------------------------------------------------
static void planner(map& map_1,
			Xstate& x_0, 
			Ustate& u_0,
			Xstate& x_goal,
			std::vector<node*>& plan,
			std::vector<node*>& tree,
			KDTree& Ktree,
			bool& reset_planner)
{
	int K = 100000;

	Xstate x_rand;
	Xstate x_prop;
	Xstate x_rewire;
	Xstate x_best;
	Xstate x_min;

	Ustate u_rewire;
	Ustate u_k; 
	Ustate u_best;
	Ustate u_min;

	bool reached = false;
	bool rewire = false;
	bool near_goal = false;
	reset_planner = false;

	double prop_time=0;
	double t_passed = 0;

	int quad = 5;
	std::uniform_int_distribution<int> n_rand_map_coordsx(x_goal[0]-quad,x_goal[0]+quad);
	std::uniform_int_distribution<int> n_rand_map_coordsy(x_goal[1]-quad,x_goal[1]+quad);

	std::vector<int> neighbors_idx;

	tree.push_back(new node(t_passed,euclidean(x_goal,x_0),nullptr,u_0,x_0));

	Ktree.Insert(tree.back());
	// K_points.push_back(tree.back());

	// Ktree.build(K_points);

	std::cout<< "Number of samples: "<< K << std::endl; 

	double accum_time = 0;
	for(int i = 0; i < K; ++i)
	{

		auto start_time = std::chrono::system_clock::now();
		// std::cout<< "Number of samples: "<< i << std::endl; 
		double prob = dist_prob(gen); //Creating random number representing probability 
		if(prob > 0.95) //Goal biasing by 5% 
		{
			x_rand = x_goal; //Assigning qrandom to be goal
		} else if(prob > 0.90)
		{
			Xstate x_r((double)n_rand_map_coordsx(gen),(double)n_rand_map_coordsy(gen),rand_theta(gen),0);
			x_rand = x_r;
		}
		else
		{
			Xstate x_r((double)rand_map_coordsx(gen)/10.0,(double)rand_map_coordsy(gen)/10.0,rand_theta(gen),0);
			x_rand = x_r;
		}
		if(x_rand[0] > map_1.width && x_rand[1] > map_1.height)
		{
			continue;
		}
		double r = L2_norm(x_rand);
		// if(tree.size() > 1)
		// {
		// 	r = std::min(L2_norm(x_rand),calc_radius(tree));
		// }
		// int nn_idx = nearest_n_idx(x_rand,tree);//Loops through the entire list for the closest neighbor
		// auto q_start_time = std::chrono::system_clock::now();

		node* q_near = Ktree.nearest_neighbor(x_rand,r); //Grabs that qnear

		// auto q_end_time = std::chrono::system_clock::now();
		// auto q_time_delay = std::chrono::duration_cast<std::chrono::nanoseconds> (q_end_time-q_start_time);
		// auto q_time_passed = q_time_delay.count()*1e-9;

		// printf("Ktree time: %f\n",q_time_passed);
		if(q_near == nullptr)
			continue;

		steer(q_near->getXstate(),x_rand,map_1,x_best,u_best,prob,near_goal);

		near_goal = false;

		node* q_new = new node(q_near->g + u_best[0]*u_best.get_tprop(),euclidean(x_goal,x_best),q_near,u_best,x_best);

		tree.push_back(q_new);
		Ktree.Insert(q_new);

		double dist2goal = euclidean(x_goal,q_new->getXstate());

		if(dist2goal < 1)
		{
			printf("Goal found at K: %d\n", i);
			// printf("Initial State: \n");
			// std::cout << x_0 << std::endl;
			// printf("Final state: \n");
			// std::cout << x_min << std::endl;
			// printf("Goal state:\n");
			// std::cout<<x_goal<< std::endl;
			getPlan(plan,tree);
			return;
		}
		else if(dist2goal < 3)
		{
			near_goal = true;
		}
		
		auto end_time = std::chrono::system_clock::now();
		auto time_delay = std::chrono::duration_cast<std::chrono::nanoseconds> (end_time-start_time);
		auto time_passed = time_delay.count()*1e-9;
		accum_time+=time_passed;
		// printf("Accumulated_time: %.4fseconds\n",accum_time);
		if(accum_time > 10.0)
		{
			reset_planner = true; 
			return;
		}
	}
	reset_planner = true;
	printf("No plan found \n");
	// CleanUp(tree);
	//no plan by default
    return;
}

struct results
{
	double time; 
	double cost;
	double node_expansions;
};
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
	int block_width[2] = {15,15};
	int coords[2] ={0,0};
	int coords_start[2]={30,20}; //30,20 ; 10,20; 5,35; 40,46;(x,y)
	int coords_goal[2]={7,46}; // 7, 46; 30,46; 40,15; 48,50; 45,10; (x,y)

	int start_x_coord_array[5] = {30,10,5,5,40};
	int start_y_coord_array[5] = {20,20,35,35,46};
	int goal_x_coord_array[5] = {7,30,40,48,45};
	int goal_y_coord_array[5] = {46,46,15,50,10};
	int x_size=0, y_size=0;

	map map_1; 
	std::vector<node*> plan; 
	std::vector<node*> tree;
	KDTree Ktree;

	Ustate u_start(0,0);
	Xstate x_start(coords_start[0],coords_start[1],calc_angle(coords_goal,coords_start),0);
	Xstate x_goal(coords_goal[0],coords_goal[1],PI/2,0);

	object husky_robot;
	object start_pos;
 	object goal_pos;
	object path; 
	tether t; 

	map_1.loadMap(argv[1]);
	map_1.calc_collision_set();

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
	
	for(int trials = 0; trials < 5; trials++)
	{
		results res[10];
		bool too_long = false;
		bool reset_planner = false; 
		int n_tries = 0;
		int succ_trial = 0;
		int s_x_c = start_x_coord_array[trials];
		int s_y_c = start_y_coord_array[trials];
		coords_start[0] = s_x_c;
		coords_start[1] = s_y_c; //30,20 ; 10,20; 5,35; 40,46;(x,y)
		int g_x_c = goal_x_coord_array[trials];
		int g_y_c = goal_y_coord_array[trials];
		coords_goal[0] = g_x_c;
		coords_goal[1] = g_y_c;
		x_start[0] = s_x_c;
		x_start[1] = s_y_c;
		x_start[2] = calc_angle(coords_goal,coords_start);
		x_goal[0] = g_x_c;
		x_goal[1] = g_y_c;
		printf("Start Coord(x,y,theta) = %.2f,%.2f,%.2f \n",x_start[0],x_start[1],x_start[2]);
		printf("Goal Coord(x,y,theta) = %.2f,%.2f,%.2f \n",x_goal[0],x_goal[1],x_goal[2]);
		while(succ_trial < 10)
		{
			auto start_time = std::chrono::system_clock::now();
			while(!too_long)
			{

				if(!tree.empty())
				{
					CleanUp(tree,Ktree);
				}
				
				if(!plan.empty())
				{
					plan.clear();
				}
				// if(!Ktree.is_empty())
				// {
				// 	// printf("KDTree empty\n");
				// 	Ktree.Clear();
				// }
				planner(map_1,x_start,u_start,x_goal,plan,tree,Ktree,reset_planner);
				if(reset_planner)
				{
					n_tries++;
					printf("Number of tries: %d \n", n_tries);
				}
				else
				{
					too_long = true;
				}
				if(n_tries >=15)
				{
					too_long = true;
				}
			}	
			n_tries = 0;
			too_long = false;
			auto end_time = std::chrono::system_clock::now();
			auto time_delay = std::chrono::duration_cast<std::chrono::nanoseconds> (end_time-start_time);
			auto time_passed = time_delay.count()*1e-9;
			res[succ_trial].time = time_passed;
			res[succ_trial].cost = plan.back()->g;
			res[succ_trial].node_expansions = tree.size();
			++succ_trial;
			// std::cout<<"--------------- RESULTS---------------"<<std::endl;
			// std::cout<<"Planning time: "<< time_passed << " seconds" << std::endl;
			// std::cout<<"Tree size:" << tree.size() << std::endl;
			// std::cout<<"Cost of the plan: " << plan.back()->g << std::endl; 
			// std::cout<<"--------------------------------------"<<std::endl;
		}
		for(int i = 0; i < succ_trial; ++i)
		{
			printf("%.4f s,",res[i].time);
		}
		printf("\n");
		for(int i = 0; i < succ_trial; ++i)
		{
			printf("%.4f ,",res[i].node_expansions);
		}
		printf("\n");
		for(int i = 0; i < succ_trial; ++i)
		{
			printf("%.4f m,",res[i].cost);
		}
		printf("\n");
	}

	// bool too_long = false; 
	// bool reset_planner = false;  	
	// int n_tries = 0;
	// auto start_time = std::chrono::system_clock::now();
	// while(!too_long)
	// {
	// 	planner(map_1,x_start,u_start,x_goal,plan,tree,Ktree,reset_planner);
	// 	if(reset_planner)
	// 	{
	// 		n_tries++;
	// 		printf("Number of tries: %d \n", n_tries);
	// 	}
	// 	else
	// 	{
	// 		too_long = true;
	// 	}
	// 	if(n_tries >=15)
	// 	{
	// 		too_long = true;
	// 	}
	// }	
	// too_long = false;
	// auto end_time = std::chrono::system_clock::now();
	// auto time_delay = std::chrono::duration_cast<std::chrono::nanoseconds> (end_time-start_time);
	// auto time_passed = time_delay.count()*1e-9;
	// std::cout<<"--------------- RESULTS---------------"<<std::endl;
	// std::cout<<"Planning time: "<< time_passed << " seconds" << std::endl;
	// std::cout<<"Tree size:" << tree.size() << std::endl;
	// std::cout<<"Cost of the plan: " << plan.back()->g << std::endl; 
	// std::cout<<"--------------------------------------"<<std::endl;

 	auto plan_size = plan.size();
	if(plan_size <= 1)
	{
		return 1;
	}

 	start_pos.setDim(block_width[0],block_width[1]);
	start_pos.setColor(255,0,0);
	start_pos.Move(coords_start[0],coords_start[1],0);

	goal_pos.setDim(block_width[0],block_width[1]);
	goal_pos.setColor(255,0,0);
	goal_pos.Move(coords_goal[0],coords_goal[1],0);

	husky_robot.setDim(20,15);
	husky_robot.setColor(255,240,10);
	husky_robot.Move(coords_start[0],coords_start[1],0);

	path.setDim(1,1);
	path.setColor(0,140,255);
	path.Move(coords_start[0],coords_start[1],0);

	t.set_anchor(coords_start[0] + 5,coords_start[1]);
	t.setDim(block_width[0],block_width[1]);
	t.setColor(0,0,0);
	
	int idx = 0;
	int w_width = 1000;
	int w_height =1000;
	// bool end_path = false; 
	double h = 0.01;
	Xstate x_k(x_start);
	Xstate x_prop;
 	std::cout << "Rendering planner" << std::endl;
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
				husky_robot.Move(coords_start[0],coords_start[1],0);
			}	

			Ustate u_k = plan[idx]->getUstate();

			double prop_time = u_k.get_tprop();
			// // Xstate x_prop(plan[idx]->getXstate());
			Xstate x_planned(plan[idx]->getXstate());
			Xstate x_prop;
			// husky_robot.Move(map_1.block_x*std::round(x_prop[0]),map_1.block_y*(map_1.height-std::round(x_prop[1])),x_prop[2]);

			// //Rendering

			// glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
			// glLoadIdentity();

			// map_1.renderMap();

			// start_pos.Draw_object();
			// start_pos.Draw_coords();
			
			// goal_pos.Draw_object();
			// goal_pos.Draw_coords();

			// husky_robot.Draw_object_Angle();

			// FsSwapBuffers();
			// FsSleep(250);

			for(int i = 0; i < sec2msec(prop_time) ; ++i)
			{
				x_prop[0] = x_k[0] + u_k[0]*cos(x_k[2])*h;
				x_prop[1] = x_k[1] + u_k[0]*sin(x_k[2])*h;
				x_prop[2] = x_k[2] + u_k[1]*h;
				x_prop[3] = 0;

				x_k = x_prop;

				husky_robot.Move(x_k[0],x_k[1],x_k[2]);
				path.Move(x_k[0],x_k[1],x_k[2]);
				t.Move(x_k[0],x_k[1],x_k[2]);
				//Rendering

				// glViewport(0,0,wid,hei);
				glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
				map_1.renderMap();

				start_pos.Draw_object();
				start_pos.Draw_coords();
				
				goal_pos.Draw_object();
				goal_pos.Draw_coords();

				husky_robot.Draw_object_Angle();

				path.Draw_Path();
				t.Draw_tether();

				FsSwapBuffers();
				// FsSleep(1);
			}
			++idx; 
		}

	CleanUp(tree,Ktree);
	plan.clear();
	return 0;
}
