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

// Some potentially helpful imports
using std::vector;
using std::array;
using std::string;
using std::runtime_error;
using std::tuple;
using std::make_tuple;
using std::tie;
using std::cout;
using std::endl;

unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
//1234 12345 123456 1234567 12345678
std::default_random_engine generator(seed);
std::uniform_real_distribution<double> distribution(0.0,2.0*PI);
std::uniform_real_distribution<double> distribution_prob(0.0,1);

/// @brief 
/// @param filepath 
/// @return map, x_size, y_size

//Added by AF
double rand_sample()
{
	double rand_ = distribution(generator);
	return rand_;
};

#include "Vertice.h"
#include "helper_func.h"
//Added by AF

tuple<double*, int, int> loadMap(string filepath) {
	std::FILE *f = fopen(filepath.c_str(), "r");
	if (f) {
	}
	else {
		printf("Opening file failed! \n");
		throw runtime_error("Opening map file failed!");
	}
	int height, width;
	if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2) {
		throw runtime_error("Invalid loadMap parsing map metadata");
	}
	
	////// Go through file and add to m_occupancy
	double* map = new double[height*width];

	double cx, cy, cz;
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			char c;
			do {
				if (fscanf(f, "%c", &c) != 1) {
					throw runtime_error("Invalid parsing individual map data");
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
	return make_tuple(map, width, height);
}

// Splits string based on deliminator
vector<string> split(const string& str, const string& delim) {   
		// https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c/64886763#64886763
		const std::regex ws_re(delim);
		return { std::sregex_token_iterator(str.begin(), str.end(), ws_re, -1), std::sregex_token_iterator() };
}


double* doubleArrayFromString(string str) {
	vector<string> vals = split(str, ",");
	double* ans = new double[vals.size()];
	for (int i = 0; i < vals.size(); ++i) {
		ans[i] = std::stod(vals[i]);
	}
	return ans;
}

bool equalDoubleArrays(double* v1, double *v2, int size) {
    for (int i = 0; i < size; ++i) {
        if (abs(v1[i]-v2[i]) > 1e-3) {
            cout << endl;
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

//-------------------------Below Added by AF----------------------------------------------------
//-------------------------BELOW RRT Functions & Code ------------------------------------------------
vertice* get_nearest_neighbor(	
	vertice*& rand_con,
	vector<vertice*>& vertex)
	{
		double min_rad = std::numeric_limits<double>::infinity();
		vertice* nearest_n;
		for(vertice* v:vertex)
			{
				double rad = euclidean(v,rand_con);
				if(rad < min_rad)
				{
					min_rad = rad;
					nearest_n = v;
				}
			}
		return nearest_n;
	}
vertice* move_eps(
	vertice*& q_near,
	vertice*& q,
	vertice* q_new,
	double eps) 
{
	int DOF = q->getDOF();
	vertice* direc = normalized_direc(q_near,q,DOF);
	for(int i = 0; i < DOF; ++i)
	{
		q_new->angles[i] = q_near->angles[i] + eps*direc->angles[i];
	}
	return q_new;
};

bool check_newconfig(
	vertice*& random_conf,
	vertice*& q_near,
	vertice* q_new,
	double* map,
	double eps,
	int numofDOFs,
	int x_size,
	int y_size)
	{
		int n = 10;
		double ang_trans[numofDOFs];
		double dist =euclidean(q_near,random_conf);
		if(dist < eps)
		{
			// If q is closer than epsilon then set qnew as q
			for (int i = 0; i < n ; i++)
				{
					for(int j = 0; j < numofDOFs ; j++)
					{
						ang_trans[j] = q_near->angles[j] + ((double)(i)/(n-1))*(random_conf->angles[j] - q_near->angles[j]);
					}
					if(!IsValidArmConfiguration(ang_trans, numofDOFs, map, x_size, y_size))
					{
						//If in the first iteration is not valid then qnew is trapped
						if(i <= 1)
						{
							return false;
						}
						//If not in the first of any other iteration. Move up to i-1 where is not trapped
						else
						{
							for(int j = 0; j < numofDOFs ; j++)
							{
								q_new->angles[j] = q_near->angles[j] + ((double)(i-1)/(n-1))*(random_conf->angles[j] - q_near->angles[j]);
							}
							return true;	
						}
						return false;
					}
				}
			for(int j = 0; j < numofDOFs ; j++)
			{
				q_new->angles[j] = random_conf->angles[j];
			}
			return true;
		}
		else
		{
			//If not closer than epsilon, interpolate to epsilon
			// qnew = move_eps(qnear,random_conf,eps);
			for (int i = 0; i < n ; i++)
			{
				for(int j = 0; j < numofDOFs ; j++)
				{
					ang_trans[j] = q_near->angles[j] + ((double)(i)/(n-1))*(q_new->angles[j] - q_near->angles[j]);
				}
				if(!IsValidArmConfiguration(ang_trans, numofDOFs, map, x_size, y_size))
				{
					//If in the first iteration is not valid then qnew is trapped
					if(i <= 1)
					{
						return false;
					}
					//If not in the first of any other iteration. Move up to i-1 where is not trapped
					else
					{
						for(int j = 0; j < numofDOFs ; j++)
						{
							q_new->angles[j] = q_near->angles[j] + ((double)(i-1)/(n-1))*(q_new->angles[j] - q_near->angles[j]);
						}
						return true;
					}
					return false;
				}
			}
			return true;
		} 
	};

void extend_RRT(
	vector<vertice*>& tree,
	vertice*& random_conf,
	vertice*& last_node,
	double* map,
	int x_size,
	int y_size,
	int numofDOFs,
	int K
)
{
	double eps = 1.4;	//Epsilon distance
	//Get top nearest neihgbor 
	vertice* nearest_n = get_nearest_neighbor(random_conf,tree);
	//Move by epsilon 
	vertice* qnew = new vertice(numofDOFs);
	qnew = move_eps(nearest_n,random_conf,qnew,eps);
	if(check_newconfig(random_conf,nearest_n,qnew,map,eps,numofDOFs,x_size,y_size))//Return qnew, and its status if either reached,trapped or advanced
	{
		tree.emplace_back(qnew);//Push back qnew to the tree 
		nearest_n->childs.emplace_back(qnew); //Create edge between qnew and nearest_n 
		qnew->childs.emplace_back(nearest_n); //Edge in both ways	
		qnew->setParent(nearest_n);//Set parent for back track 
		qnew->g = nearest_n->g + euclidean(nearest_n,qnew); //Calculatint cost
		if(equalDoubleArrays(qnew->angles,random_conf->angles,numofDOFs))
			{
				qnew->state = 0; //Reached
				last_node = qnew;
			}
		else
			{
				qnew->state = 1; //Advanced
				last_node = qnew;
			}
	}
	else
	{
		nearest_n->state = 2; //Trapped
		last_node = nearest_n;
		delete qnew; //We can safely delete this variable if its trapped as it will not be used. 
	}
};

void Build_RRT(
	vector<vertice*>& tree,
	double* start_config,
	double* end_config,
	double* map,
	int K,
	int numofDOFs,
	int x_size,
	int y_size,
	bool& reached)
{
	vertice* start_vert = new vertice(start_config,K,numofDOFs); //Creating the start position as a vertice node 
	vertice* goal_vert = new vertice(end_config,K+1,numofDOFs); // Creating goal position as a vertice node
	start_vert->g = 0; //Initializing the start cost 
	tree.emplace_back(start_vert); //Pushing back start node into the tree to initialize search 
	vertice* last_node; //This will give the last node status given by check_new config 
	for(int i = 0; i < K ; ++i)
	{
		vertice* random_config = new vertice(i,numofDOFs); //Creating a vertice with random angles position
		double prob = (double)distribution_prob(generator); //Creating random number representing probability 
		if(prob > 0.95) //Goal biasing by 5% 
		{
			random_config = goal_vert; //Assigning qrandom to be goal
		}
		extend_RRT(tree,random_config,last_node,map,x_size,y_size,numofDOFs,K);
		//Check if goal is reached
		if(equalDoubleArrays(tree.back()->angles,end_config,numofDOFs))
			{
 				cout<<"GOAL Reached"<<endl; 
				reached = true;
				return;
			}
	}
	cout<<"Run out of samples"<< endl;
	cout<<"Goal not founded" << endl;
	return;
};

void ComputePathRRT(
	vector<vertice*>& path_,
	vector<vertice*>& tree,
	double* start_config,
	double* end_config,
	double* map,
	int numDOF,
	int K,
	int x_size,
	int y_size)
{
	bool reached = false;//Initializing boolean to see if the goal was reached
	Build_RRT(tree,start_config,end_config,map,K,numDOF,x_size,y_size,reached);	//Builds the tree from the start to the goal position
	if(reached) //If goal its reached then execute backtrack
	{
		path_.emplace_back(tree.back());
		while(!(path_.back()->parent == nullptr))//Stopping until the parent is nullptr because it means is the start node
		{
			vertice* father = path_.back() -> parent;
			path_.emplace_back(father);
		}
		cout<<"Exporting computed path" << endl;
	}
	else
	{
		cout<<"Path not founded :(" << endl;
	}
};
//-------------------------ABOVE RRT Functions & Code ------------------------------------------------
//-------------------------Above Added by AF----------------------------------------------------


static void planner(
			double* map,
			int x_size,
			int y_size,
			double* armstart_anglesV_rad,
			double* armgoal_anglesV_rad,
            int numofDOFs,
            double*** plan,
            int* planlength,
			int whichplanner,
			planner_results* res)
{
	//no plan by default
	vector<vertice*> plan_vec; //Vector containing the plan delivered by any algorithm
	vector<vertice*> vertx; // Vector of vertices
	vector<vertice*> vertx2;// Vector of vertices(RRT Connect)
	*plan = NULL;
	*planlength = 0;
	int K = 100000; // Initial number of samples
	int i,j;
	int countNumInvalid = 0; 
	int iter = 1; // Initializing iterator for loop
	int max_iter = 5; // Maximum number of iterations when running planner

	bool notvalid = false; // Set to TRUE if desired random start & goal configurations
	while(notvalid)	
	{
		for(int i = 0; i<numofDOFs; ++i)
		{
			armstart_anglesV_rad[i] = rand_sample();
			armgoal_anglesV_rad[i] = rand_sample();
		}
		if(IsValidArmConfiguration(armstart_anglesV_rad, numofDOFs, map, x_size, y_size) && IsValidArmConfiguration(armgoal_anglesV_rad, numofDOFs, map, x_size, y_size))
			notvalid = false;
		else 
			continue;
	} 

	cout<<"Start Position"<<endl; //Printing Start Position Angles 
	for(int i = 0; i < numofDOFs; ++i)
	{
		cout<<armstart_anglesV_rad[i]<<", ";
	}
	cout<<endl;

	cout<<"Goal Position"<< endl; //Printing Goal Position Angles
		for(int i = 0; i < numofDOFs; ++i)
	{
		cout<<armgoal_anglesV_rad[i]<<", ";
	}
	cout<<endl;

	// int tries = 0
	// Planner
	while(plan_vec.empty())
	// for(int tries = 0; tries < 5; ++tries)
	{
		auto start_time = std::chrono::system_clock::now();
		vertx.clear(); // Clear all the vector from previous fails
		vertx2.clear();
		plan_vec.clear();	
		switch (whichplanner)
		{
		case RRT: // 0 = RRT 
			cout<<"Planner Selected: RRT"<<endl;
			ComputePathRRT(plan_vec,vertx,armstart_anglesV_rad,armgoal_anglesV_rad,map,numofDOFs,K,x_size,y_size);
			break;
		default:
			cout<<"Please Stop Program and select a correct planner"<<endl;
			break;
		}
		if(iter >= max_iter)
		{
			cout<<"No solution after" << iter << "iterations :("<< endl;
			K += 10000;
			cout<<"Running planner again with " << K << " samples" << endl;
			iter = 0;
		}
		if(plan_vec.empty())
		{
			cout<< "Running planner " << max_iter-iter <<" more times." << endl;
			++iter;
		}
		auto end_time = std::chrono::system_clock::now();
    	auto time_delay = std::chrono::duration_cast<std::chrono::nanoseconds> (end_time-start_time);
		float time_passed = time_delay.count()*1e-9;
		res[iter].time = time_passed;
		res[iter].cost = plan_vec.front()->g;
		res[iter].success_less_5 = (time_passed<=5) ? true:false;
		res[iter].tree_size = vertx.size();
		res[iter].tree_b_size = (!vertx2.empty()) ? vertx2.size(): 0;
	}
	// Formatting results
	cout<<"--------------- RESULTS---------------"<<endl;
	cout<<"Planning time: "<< res[iter].time << " seconds" << endl;
	if(whichplanner == 1)
	{
		cout<<"Vertices generated in tree A: " << res[iter].tree_size << endl;
		cout<<"Vertices generated in tree B: " << res[iter].tree_b_size << endl;
	}
	else
		cout<<"Average vertices generated in graph " << res[iter].tree_size << endl;
	cout<<"Path cost is " << res[iter].cost << endl;

    // Computed Path into .txt
	*plan = (double**) malloc(plan_vec.size()*sizeof(double*));
    for (i = 0; i < plan_vec.size(); i++){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
        for(j = 0; j < numofDOFs; j++){
            (*plan)[i][j]= plan_vec[plan_vec.size()-(i+1)]->angles[j];
        }
        if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size)) {
			++countNumInvalid;
        }
    }
	printf("Linear interpolation collided at %d instances across the path\n", countNumInvalid);
	*planlength = plan_vec.size();

	if(!vertx.empty())
	{
		for(int i = 0; i < vertx.size()-1; ++i)
		{
			delete vertx[i];
		}
	}
	if(!vertx2.empty())
	{
		for(int i = 0; i < vertx2.size()-1; ++i)
		{
			delete vertx2[i];
		}
	}
	// if(!plan_vec.empty())
	// {
	// 	for(int i = 0; i < plan_vec.size()-1; ++i)
	// 	{
	// 		delete plan_vec[i];
	// 	}
	// }
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
	tie(map, x_size, y_size) = loadMap(argv[1]);
	const int numOfDOFs = std::stoi(argv[2]);
	double* startPos = doubleArrayFromString(argv[3]);
	double* goalPos = doubleArrayFromString(argv[4]);
	int whichPlanner = std::stoi(argv[5]);
	string outputFile = argv[6];

	if(!IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size)||
			!IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size)) {
		throw runtime_error("Invalid start or goal configuration!\n");
	}

	///////////////////////////////////////
	//// Feel free to modify anything below. Be careful modifying anything above.

	double** plan = NULL;
	int planlength = 0;
	planner_results res[5]; 
	planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength,whichPlanner,res);

	//// Feel free to modify anything above.
	//// If you modify something below, please change it back afterwards as my 
	//// grading script will not work and you will recieve a 0.
	///////////////////////////////////////

    // Your solution's path should start with startPos and end with goalPos
    if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || 
    	!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs)) {
		throw std::runtime_error("Start or goal position not matching");
	}

	/** Saves the solution to output file
	 * Do not modify the output log file output format as it is required for visualization
	 * and for grading.
	 */
	std::ofstream m_log_fstream;
	m_log_fstream.open(outputFile, std::ios::trunc); // Creates new or replaces existing file
	if (!m_log_fstream.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	m_log_fstream << argv[1] << endl; // Write out map name first
	/// Then write out all the joint angles in the plan sequentially
	for (int i = 0; i < planlength; ++i) {
		for (int k = 0; k < numOfDOFs; ++k) {
			m_log_fstream << plan[i][k] << ",";
		}
		m_log_fstream << endl;
	}

	//Printing results if running multiple times:
	std::ofstream m_result_fstream;
	m_result_fstream.open("results.txt", std::ios::trunc);
	for(int i = 0; i < 5;++i)
	{
		m_result_fstream << "Run#" << i << endl;
		m_result_fstream << "Planning Time: " << res[i].time << endl;
		m_result_fstream << "Vertices generated in graph a: " << res[i].tree_size << endl;
		m_result_fstream << "Vertices generated in graph b: " << res[i].tree_b_size << endl;
		m_result_fstream << "Path Cost:" << res[i].cost << endl;
		m_result_fstream << "Succes rate under 5 secs: " << res[i].success_less_5 << endl;
		m_result_fstream << endl;
		m_result_fstream << endl;
	}
}
