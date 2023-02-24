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
#include "node.h" 
  

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

std::uniform_real_distribution<double> rand_u_vel(-1.0,1.0);
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
	for(int i = 0; i < x_goal.size(); i+=2)
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
	
	for(int i = 0; i < K; ++i)
	{
		std::cout<< "K: "<< i << std::endl; 
		double prob = dist_prob(gen); //Creating random number representing probability 
		if(prob > 0.95) //Goal biasing by 5% 
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
				u_k[0] = rand_u_vel(gen);
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
				printf("Goal found \n");
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
	double* map;
	int x_size, y_size;
	std::tie(map, x_size, y_size) = loadMap(argv[1]);
	std::vector<node*> plan; 
	std::vector<node*> tree;
	Ustate u_start(0,0);
	Xstate x_prop;
	Xstate x_start(0,0,0,0);
	Xstate x_goal(2,0.1,0.3,0);
	u_start.set_tprop(1);
	std::cout << calc_radius() << std::endl;
	// double prop_time = rand_t_prop(gen);
	// auto a = x_goal.getPointer();
	// printf("Testing printf\n");
	// printf("%f,%f,%f,%f \n",a[0],a[1],a[2],a[3]);
	// printf("Initial state\n");
	// std::cout<<x_start << std::endl;
	// printf("Goal State\n");
	// std::cout << x_goal<< std::endl;
	// printf("Before prop\n");
	// std::cout << x_prop << std::endl;
	// printf("After prop\n");
	// x_prop.propagate(x_start,u_start);
	// std::cout << x_prop << std::endl;
	// std::cout <<x_start << std::endl;
	// nodeHdle test(new node(1,1,nullptr,u_start));
	// test.node_p->setXstate(x_goal);
	planner(map,x_size,y_size,x_start,u_start,x_goal,plan,tree);

	CleanUp(tree);
	// CleanUp(plan);
	plan.clear();
	//// Feel free to modify anything above.
	//// If you modify something below, please change it back afterwards as my 
	//// grading script will not work and you will recieve a 0.
	///////////////////////////////////////

    // Your solution's path should start with startPos and end with goalPos
    // if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || 
    // 	!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs)) {
	// 	throw std::runtime_error("Start or goal position not matching");
	// }
	return 0;
}
