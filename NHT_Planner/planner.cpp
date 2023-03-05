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

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

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


unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
//1234 12345 123456 1234567 12345678
std::default_random_engine gen(seed);

std::uniform_real_distribution<double> rand_t_prop(0.0,5.0);

std::uniform_int_distribution<int> rand_u_vel(-100,100);
std::uniform_real_distribution<double> rand_u_ang_vel(-2.0,2.0);

std::uniform_real_distribution<double> rand_xdot(-1.0,1.0);
std::uniform_real_distribution<double> rand_x(-5.0,5.0);
std::uniform_real_distribution<double> rand_theta(-2.0*PI,2.0*PI);
std::uniform_real_distribution<double> rand_thetadot(-1.0,1.0);

std::uniform_real_distribution<double> dist_prob(0.0,1);


/// @brief 
/// @param filepath 
/// @return map, x_size, y_size


// #include "Vertice.h"

//Added by AF

std::tuple<double*, int, int> loadMap(std::string filepath) {
	std::FILE *f = fopen(filepath.c_str(), "r");
	if (f) {
	}
	else {
		printf("Opening file failed! \n");
		throw std::runtime_error("Opening map file failed!");
	}
	int height, width;
	if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2) {
		throw std::runtime_error("Invalid loadMap parsing map metadata");
	}
	
	////// Go through file and add to m_occupancy
	double* map = new double[height*width];

	double cx, cy, cz;
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			char c;
			do {
				if (fscanf(f, "%c", &c) != 1) {
					throw std::runtime_error("Invalid parsing individual map data");
				}
			} while (isspace(c));
			if (!(c == '0')) { 
				map[y+x*width] = 1; // Note transposed from visual
			} else {
				map[y+x*width] = 0;
			}
		}
	}
	fclose(f);
	return std::make_tuple(map, width, height);
}

// Splits string based on deliminator
std::vector<std::string> split(const std::string& str, const std::string& delim) {   
		// https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c/64886763#64886763
		const std::regex ws_re(delim);
		return { std::sregex_token_iterator(str.begin(), str.end(), ws_re, -1), std::sregex_token_iterator() };
}


double* doubleArrayFromString(std::string str) {
	std::vector<std::string> vals = split(str, ",");
	double* ans = new double[vals.size()];
	for (int i = 0; i < vals.size(); ++i) {
		ans[i] = std::stod(vals[i]);
	}
	return ans;
}

bool equalDoubleArrays(double* v1, double *v2, int size) {
    for (int i = 0; i < size; ++i) {
        if (abs(v1[i]-v2[i]) > 1e-3) {
            std::cout << std::endl;
            return false;
        }
    }
    return true;
}

typedef struct {
	int X1, Y1;
	int X2, Y2;
	int Increment;
	int UsingYIndex;
	int DeltaX, DeltaY;
	int DTerm;
	int IncrE, IncrNE;
	int XIndex, YIndex;
	int Flipped;
} bresenham_param_t;


void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size) {
	double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}


void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params) {
	params->UsingYIndex = 0;

	if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
		(params->UsingYIndex)++;

	if (params->UsingYIndex)
		{
			params->Y1=p1x;
			params->X1=p1y;
			params->Y2=p2x;
			params->X2=p2y;
		}
	else
		{
			params->X1=p1x;
			params->Y1=p1y;
			params->X2=p2x;
			params->Y2=p2y;
		}

	 if ((p2x - p1x) * (p2y - p1y) < 0)
		{
			params->Flipped = 1;
			params->Y1 = -params->Y1;
			params->Y2 = -params->Y2;
		}
	else
		params->Flipped = 0;

	if (params->X2 > params->X1)
		params->Increment = 1;
	else
		params->Increment = -1;

	params->DeltaX=params->X2-params->X1;
	params->DeltaY=params->Y2-params->Y1;

	params->IncrE=2*params->DeltaY*params->Increment;
	params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
	params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

	params->XIndex = params->X1;
	params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y) {
	if (params->UsingYIndex) {
        *y = params->XIndex;
        *x = params->YIndex;
        if (params->Flipped)
            *x = -*x;
    }
	else {
        *x = params->XIndex;
        *y = params->YIndex;
        if (params->Flipped)
            *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params) {
	if (params->XIndex == params->X2) {
        return 0;
    }
	params->XIndex += params->Increment;
	if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
		params->DTerm += params->IncrE;
	else {
        params->DTerm += params->IncrNE;
        params->YIndex += params->Increment;
	}
	return 1;
}

int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
			 int x_size, int y_size) {
	bresenham_param_t params;
	int nX, nY; 
	short unsigned int nX0, nY0, nX1, nY1;

	//printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
		
	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

	//printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
			return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
			 int x_size, int y_size) {
    double x0,y0,x1,y1;
    int i;

	 //iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
	y1 = 0;
	for(i = 0; i < numofDOFs; i++){
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
			return 0;
	}    
	return 1;
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
	double r = 1.5;
	int min_node_idx = 0;
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
			else
			{
				min_node_idx = -1;
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

//-------------------------Below Added by AF----------------------------------------------------
//-------------------------BELOW RRT Functions & Code ------------------------------------------------
static void planner(
			double* map,
			int x_size,
			int y_size,
			Xstate& x_0, 
			Ustate& u_0,
			Xstate& x_goal,
			std::vector<node*>& plan,
			std::vector<node*>& tree)
{
	int K = 100000;
	Xstate x_rand;
	Xstate x_prop;
	Xstate x_min;
	Ustate u_k;
	Ustate u_min;
	bool reached = false;
	double prop_time=0;

	tree.push_back(new node(0,euclidean(x_goal,x_0),nullptr,u_0,x_0));
	std::cout<< "Number of samples: "<< K << std::endl; 
	for(int i = 0; i < K; ++i)
	{
		// std::cout<< "Number of samples: "<< i << std::endl; 
		double prob = dist_prob(gen); //Creating random number representing probability 
		if(prob > 0.20) //Goal biasing by 5% 
		{
			x_rand = x_goal; //Assigning qrandom to be goal
		}
		else
		{
			Xstate x_r(rand_x(gen),rand_xdot(gen),rand_theta(gen),rand_thetadot(gen));
			x_rand = x_r;
		}

		int nn_idx = nearest_n_idx(x_rand,tree);//Loops through the entire list for the closest neighbor

		if(nn_idx >= 0)
		{
			Xstate x_near = tree[nn_idx]->getXstate(); //Grabs that qnear
			int count = 0;
			double min_dist = std::numeric_limits<double>::infinity(); 			
			for(int i = 0; i < 20000; ++i)
			{
				u_k[0] = (double) rand_u_vel(gen)/100.0;
				u_k[1] = rand_u_ang_vel(gen);
				u_k.set_tprop(rand_t_prop(gen));
 

				x_prop.propagate(x_near,u_k);

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
			tree.push_back(new node(1,euclidean(x_goal,x_min),tree[nn_idx],u_min,x_min)) ;
			double dist2goal = euclidean(x_goal,tree.back()->getXstate());
			if(dist2goal < 0.1)
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
	/*
	TODO:
		-Figure out the best way to return the control input and time of propagation

		-How to do goal biasing? As of now, I can generate random points in the map and 
		connect them. Having a maximum time of propagation of 5 secs
			-Pick the minimum. Out of 10.000 propagations. pick the closes to the "goal"
	*/
	printf("No plan found \n");
	// CleanUp(tree);
	//no plan by default
    return;
}

void DrawTarget(int x,int y,int w,int h)
{
	glColor3ub(255,200,10);
	glBegin(GL_QUADS);
	glVertex2i(x  ,y); //0,0
	glVertex2i(x+w,y); //1,0
	glVertex2i(x+w,y+h); // 1,1
 	glVertex2i(x  ,y+h); // 0,1
	glEnd();
}

void DrawTarget_angle(int x,int y,int w,int h,double angle)
{
	double mag = sqrt(w*w + h*h);
	glColor3ub(255,0,255);
	glBegin(GL_QUADS);
	glVertex2i(x,y); //0,0
	glVertex2i(x+w*cos(angle),y+w*sin(angle)); //1,0
	glVertex2i(x+mag*cos(0.785398+angle),y+mag*sin(0.785398+angle)); // 1,1
 	glVertex2i(x-h*sin(angle),y+h*cos(angle)); // 0,1
	glEnd(); 
}

void DrawPOS(int x,int y,int w,int h)
{
	glColor3ub(255,0,0);
	glBegin(GL_QUADS);
	glVertex2i(x  ,y); //0,0
	glVertex2i(x+w,y); //1,0
	glVertex2i(x+w,y+h); // 1,1
 	glVertex2i(x  ,y+h); // 0,1
	glEnd();
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

void polar2meter(double* coords,double x,double theta)
{
	coords[0] = std::round((x*cos(theta))/0.01);
	coords[1] = std::round((x*sin(theta))/0.01);
}
void print_pos(Xstate& x)
{
	double coords[2]={0,0};
	polar2coord(coords,x); 
	std::cout<< std::round(coords[0]/0.01) << "," << std::round(coords[1]/0.01) << std::endl;
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
int main(int argc, char ** argv) {
	double* map = nullptr;
	int x_size=0, y_size=0;
	// std::tie(map, x_size, y_size) = loadMap(argv[1]);
	std::vector<node*> plan; 
	std::vector<node*> tree;
	Ustate u_start(0,0);
	Xstate x_prop;
	Xstate x_start(0,0,0,0);
	printf("Initial condition in X,Y:\n");
	print_pos(x_start);
	Xstate x_goal(2,0,0.3,0);
	printf("Goal condition in X,Y: \n");
	print_pos(x_goal);
	u_start.set_tprop(0);
	printf("radius to test near neighbors:\n");
	std::cout << calc_radius() << std::endl;
	planner(map,x_size,y_size,x_start,u_start,x_goal,plan,tree);
	int plan_size = plan.size();
	double coords[2] ={0,0};
	double coords_start[2]={0,0};
	polar2meter(coords_start,x_start[0],x_start[2]);
	double coords_goal[2]={0,0};
	polar2meter(coords_goal,x_goal[0],x_goal[2]);
	int idx = 1; 
	//// Feel free to modify anything above.
	//// If you modify something below, please change it back afterwards as my 
	//// grading script will not work and you will recieve a 0.
	///////////////////////////////////////

	// for(int i = 0; i < plan_size ; ++i)
	// {
	// 	Xstate state(plan[i]->getXstate());
	// 	print_pos(state);
	// }

	Xstate x_k(plan[0]->getXstate());

	int w_width = 1200;
	int w_height = 1200;

	FsOpenWindow(0,0,w_width,w_height,1);


		for(;;)
		{
			FsPollDevice();
			if(FSKEY_ESC==FsInkey())
			{
				break;
			}
			if(idx >= plan_size)
				idx = 0;

			Ustate u_k(plan[idx]->getUstate());
			Xstate x_prop(plan[idx]->getXstate()); 
			double coords[2] ={0,0};
			// Ustate u(plan[idx]->getUstate());
			polar2coord(coords,x_prop); 
			meter2pixel(coords);
			int t_prop = sec2msec(u_k.get_tprop());
			glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
			DrawPOS(w_width/2+coords_start[0],w_height/2+coords_start[1],20,20);
			DrawPOS(w_width/2+coords_goal[0],w_height/2+coords_goal[1],20,20);
			DrawTarget(w_width/2+coords[0],w_height/2+coords[1],10,15);
			// DrawTarget_angle(400+coords[0],400+coords[1],10,15,x[2]);
			FsSwapBuffers();
			FsSleep(250);

			// for(int i = 0; i < t_prop ; ++i)
			// {

			// 	// printf("Iteration %d \n",i);
			// 	// x_k = din_sytem.A_dt*x_k + din_sytem.B_dt*u_k;
			// 	x_prop[0] = x_k[0] + 0.007592*x_k[1] + 0.001579*u_k[0]; 
			// 	x_prop[1] = 0.5606*x_k[1] + 0.2882*u_k[0];
			// 	x_prop[2] = x_k[2] + 0.001705*x_k[3] + 0.008201*u_k[1];
			// 	x_prop[3] = 0.002881*x_k[3] + 0.9858*u_k[1];
			// 	// x_k[0] = x_prop[0];
			// 	// x_k[1] = x_prop[1];
			// 	// x_k[2] = x_prop[2];
			// 	// x_k[3] = x_prop[3];
			// 	// std::cout << x_k << std::endl;
			// 	x_k = x_prop; 

			// 	double coords[2] ={0,0};
			// 	// Ustate u(plan[idx]->getUstate());
			// 	polar2coord(coords,x_prop); 
			// 	meter2pixel(coords);
			// 	// std::cout<< coords[0] << "," << coords[1] << std::endl;
			// };

			++idx; 
		}
    // Your solution's path should start with startPos and end with goalPos
    // if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || 
    // 	!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs)) {
	// 	throw std::runtime_error("Start or goal position not matching");
	// }

	CleanUp(tree);
	plan.clear();
	return 0;
}
