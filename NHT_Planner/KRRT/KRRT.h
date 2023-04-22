#include <math.h>
#include <random>
#include <vector>
#include <algorithm>
#include <chrono>
#include <string>
#include <iostream> // cout, endl
#include <fstream> // For reading/writing files
#include <limits>
#include <node.h>
#include <map.h>
#include <object.h>
#include <KDtree.h>

struct results
{
	double time; 
	double cost;
	double node_expansions;
    double beta;
};

class KRRT
{
    public:
        bool render = false;
        double* map_t = nullptr;
        double h = 0.01;
        double c_pi= 3.141592653589793;
        double eps = 0.1;
        double alpha = 1.0;
        double time2exit = 10.0;
        double weights[4] = {1.0,1.0,1.0,1.0};

        int coords[2] ={0,0};
        int coords_start[2]={30,20}; //30,20 ; 10,20; 5,35; 40,46;(x,y)
        int coords_goal[2]={7,46}; // 7, 46; 30,46; 40,15; 48,50; 45,10; (x,y)
        int K = 200000;
        int n_scenarios = 5;
        int tolerance = 3;
        static const int n_trials = 10;

        int start_x_coord_array[5] = {30,10,5,5,40};
        int start_y_coord_array[5] = {20,20,35,35,46};
        int goal_x_coord_array[5] = {7,30,40,48,45};
        int goal_y_coord_array[5] = {46,46,15,50,10};
        int block_width[2] = {15,15};

        int x_size=0, y_size=0;
        int idx = 0;
        int w_width = 1000;
        int w_height =1000;

        map map_1;

        std::vector<node*> plan; 
        std::vector<node*> tree;
        std::vector<results> data;

        Ustate u_start;
        Ustate u_k;

        Xstate x_start; 
        Xstate x_goal;
        Xstate x_planned;
        Xstate x_prop;
        Xstate x_p;

        KDTree Ktree;

        object husky_robot;
        object start_pos;
        object goal_pos;
        object path; 
        tether t; 

        unsigned int seed;
        std::default_random_engine gen;
        
        std::fstream myfile;

        inline int get_map_idx(double x,double y,int mode)
        {
            if(mode == 1)
            {
                return(map_1.height-y-1)*map_1.width + x;
            }
            if(mode == 2)
            {
                return  y*map_1.height + x;
            }

            return 0;
        }

        double euclidean(Xstate& x_goal,const Xstate& x_near);
        double euclidean(double xf,double yf,double xi,double yi);
        double L2_norm(const Xstate& x_s);
        double LQR_Cost(Xstate& x_k,Ustate& u_k);
        double calc_radius(std::vector<node*>& tree);
        double calc_angle(double xf,double yf,double xi,double yi);
        double calc_angle(int xf[],int xi[]);

        int nearest_n_idx(Xstate x_rand,std::vector<node*>& tree);
        int getPlanSize();
        int sec2msec(double sec);
        
        void nearest_nn_idx(Xstate x_rand,double r,std::vector<node*>& tree,std::vector<int>& nn_idxs);
        void CleanUp(std::vector<node*>& tree, KDTree& Ktree);
        void getPlan(std::vector<node*>& plan,node* q_last);
        void map2block(double *map_coords,map map_1);
        void steer(Xstate& x_near,Xstate& x_rand,map map_1,Xstate& x_best,Ustate& u_best, double prob,bool near_goal);
        void Add_Edge(node *q_min,node *q_near);
        void updte_pos_obj(const Xstate& inc_x);
        void draw_obj();
        void UpdateControl();
        void Initialize();
        
        bool planner();
        bool ObstacleFree(Xstate& x_near,Xstate& x_rand,map map_1,Xstate& x_best,Ustate& u_best, double prob,bool near_goal);

        Xstate propagate(Xstate& i_x_k, Ustate& u_k,double *map, int x_size, int y_size);
        Xstate propagate_one_step(Xstate& inc_x,Ustate& inc_u);

        Xstate dynamics (const Xstate& x, const Ustate& u, double h);
        Xstate rk4step(const Xstate& x, const Ustate& u, double h);

    public:
        
        bool plan_trials();
        bool one_shot_plan();

        void Render();
        KRRT()
        {
            Initialize();
            seed = std::chrono::system_clock::now().time_since_epoch().count();
            std::cout << seed << std::endl;
            // gen.seed(seed);
            gen.seed(14968483);  
        };
        ~KRRT()
        {
            CleanUp(tree,Ktree);
	        plan.clear();
        }
        void LoadMap(std::string filepath)
        {
            map_1.loadMap(filepath);
            map_1.calc_collision_set();
        }
        inline void saveResults(char* file_path)
        {
            std::ofstream myFile(file_path);
        };

        bool def_start_pos(Xstate& inc_x);
        bool def_goal_pos(Xstate& inc_x);
        void set_objects();
        bool ResetPos();
};

