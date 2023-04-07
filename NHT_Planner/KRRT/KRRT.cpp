#ifndef KiRTT    
    #define KiRTT
    #include "KRRT.h"
    void KRRT::Initialize()
    {
        u_start.setState(0,0,0);
        x_start.setState(coords_start[0],coords_start[1],calc_angle(coords_goal,coords_start),0);
        x_goal.setState(coords_goal[0],coords_goal[1],c_pi/2,0);
    };
    bool KRRT::def_start_pos(Xstate& inc_x)
    {
        this->x_start.setState(inc_x);
        int start_idx = get_map_idx(inc_x[0],inc_x[1],1);
        if(this->map_1.map_ptr[start_idx] == 1)
        {
            printf("Invalid start location\n");
            return false;
        }
        else
        {
            return true;
        }
    }
    bool KRRT::def_goal_pos(Xstate& inc_x)
    {
        this->x_goal.setState(inc_x);
        int goal_idx = get_map_idx(inc_x[0],inc_x[1],1);
        if(this->map_1.map_ptr[goal_idx] == 1)
        {
            printf("Invalid Goal location \n");
            return false;
        }
        else 
        {
            return true;
        }
    }
    double KRRT::euclidean(Xstate& x_goal,const Xstate& x_near)
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
    double KRRT::euclidean(double xf,double yf,double xi,double yi)
    {
        return sqrt(pow(xf-xi,2)+pow(yf-yi,2));
    }
    double KRRT::euclidean(double *coord_f,double *coord_b)
    {
        return sqrt(pow(coord_f[0]-coord_b[0],2)+pow(coord_f[1]-coord_b[0],2));
    }
    double KRRT::L2_norm(Xstate& x_s)
    {
        double sum = 0;
        for(int i = 0; i < x_s.size(); ++i )
        {
            sum+=pow(x_s[i],2);
        }
        return sqrt(sum);
    }
    double KRRT::LQR_Cost(Xstate& x_k, Ustate& u_k)
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
    double KRRT::calc_radius(std::vector<node*>& tree)
    {
        double d = 3; 
        double gamma = 500;
        double delta = (PI*PI)/2;
        double V = tree.size();
        return (gamma/delta)*pow((log(V)/V),1/d);
    }

    int KRRT::nearest_n_idx(Xstate x_rand,std::vector<node*>& tree)
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
    void KRRT::nearest_nn_idx(Xstate x_rand,double r,std::vector<node*>& tree,std::vector<int>& nn_idxs)
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
    void KRRT::CleanUp(std::vector<node*>& tree, KDTree& Ktree)
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
    void KRRT::getPlan(std::vector<node*>& plan,std::vector<node*>& tree)
    {
        for(node* p = tree.back(); p != nullptr; p = plan.back()->getParent())
        {
            plan.push_back(p);
        }
        reverse(plan.begin(),plan.end());
    };
    double KRRT::calc_angle(double xf,double yf,double xi,double yi)
    {	
        double angle = atan2(yf-yi,xf-xi);
        if(angle < 0)
        {
            angle = 2*PI+angle;
        }
        return angle;

    }
    double KRRT::calc_angle(int xf[],int xi[])
    {	
        double angle = atan2(xf[1]-xi[1],xf[0]-xi[0]);
        if(angle < 0)
        {
            angle = 2*PI+angle;
        }
        return angle;
    }
    void KRRT::map2block(double *map_coords,map map_1)
    {
        map_coords[0] = map_1.block_x*std::round(map_coords[0]);
        map_coords[1] = map_1.block_y*(map_1.height - std::round(map_coords[1]));
    }
    void KRRT::steer(Xstate x_near,Xstate x_rand,map map_1,Xstate& x_best,Ustate& u_best, double prob,bool near_goal)
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
    bool KRRT::planner()
    {
        Xstate x_rand;
        Xstate x_prop;
        Xstate x_best;

        Ustate u_k; 
        Ustate u_best;

        bool near_goal = false;
        
        double prop_time=0;
        double t_passed = 0;
        double accum_time = 0;

        int quad = 5;

        auto seed = std::chrono::system_clock::now().time_since_epoch().count();
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

        std::uniform_int_distribution<int> n_rand_map_coordsx(x_goal[0]-quad,x_goal[0]+quad);
        std::uniform_int_distribution<int> n_rand_map_coordsy(x_goal[1]-quad,x_goal[1]+quad);

        tree.push_back(new node(t_passed,euclidean(x_goal,x_start),nullptr,u_start,x_start));

        Ktree.Insert(tree.back());

        std::cout<< "Number of samples: "<< K << std::endl; 

        for(int i = 0; i < K; ++i)
        {

            auto start_time = std::chrono::system_clock::now();
            double prob = dist_prob(gen); //Creating random number representing probability 
            if(prob > 0.95) //Goal biasing by 5% 
            {
                x_rand = x_goal; //Assigning qrandom to be goal
            } else if(prob > 0.80)
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
            int nn_idx = nearest_n_idx(x_rand,tree);//Loops through the entire list for the closest neighbor

            node* q_near = Ktree.nearest_neighbor(x_rand,r); //Grabs that qnear

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
                return true;
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
                return false;
            }
        }
        printf("No plan found \n");
        // CleanUp(tree);
        //no plan by default
        return false; 
    }
    bool KRRT::plan_trials()
    {
        for(int trials = 0; trials < n_scenarios; trials++)
        {
            results res[n_trials];
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

            x_start.setState(s_x_c,s_y_c,calc_angle(coords_goal,coords_start),0);
            x_goal.setState(g_x_c,g_y_c,c_pi/2,0);

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

                    bool success = planner();
                    if(!success)
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
                printf("%.4f,",res[i].time);
            }
            printf("\n");
            for(int i = 0; i < succ_trial; ++i)
            {
                printf("%.4f,",res[i].node_expansions);
            }
            printf("\n");
            for(int i = 0; i < succ_trial; ++i)
            {
                printf("%.4f,",res[i].cost);
            }
            printf("\n");
        }
    }
    bool KRRT::one_shot_plan()
    {
        bool too_long = false; 
        bool reset_planner = false;  	
        int n_tries = 0;
        auto start_time = std::chrono::system_clock::now();
        while(!too_long)
        {
            bool success = planner();
            if(!success)
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
        too_long = false;
        auto end_time = std::chrono::system_clock::now();
        auto time_delay = std::chrono::duration_cast<std::chrono::nanoseconds> (end_time-start_time);
        auto time_passed = time_delay.count()*1e-9;
        std::cout<<"--------------- RESULTS---------------"<<std::endl;
        std::cout<<"Planning time: "<< time_passed << " seconds" << std::endl;
        std::cout<<"Tree size:" << tree.size() << std::endl;
        std::cout<<"Cost of the plan: " << plan.back()->g << std::endl; 
        std::cout<<"--------------------------------------"<<std::endl;
    }
    void updte_pos_obj(const Xstate& inc_x);
    {
        husky_robot.Move(inc_x[0],inc_x[1],inc_x[2]);
        path.Move(inc_x[0],inc_x[1],inc_x[2]);
        t.Move(inc_x[0],inc_x[1],inc_x[2]);
    }
    void KRRT::draw_obj()
    {
        map_1.renderMap();

        start_pos.Draw_object();
        start_pos.Draw_coords();
        
        goal_pos.Draw_object();
        goal_pos.Draw_coords();

        husky_robot.Draw_object_Angle();

        path.Draw_Path();
        t.Draw_tether();
    }
    void KRRT::Render()
    {
        int block_width[2] = {15,15};
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
        double h = 0.01;
        // bool end_path = false; 
        Xstate x_k(x_start);
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

            u_k = plan[idx]->getUstate();

            double prop_time = u_k.get_tprop();
            // // Xstate x_prop(plan[idx]->getXstate());
            // x_planned = plan[idx]->getXstate();
            Xstate x_prop;
            for(int i = 0; i < sec2msec(prop_time) ; ++i)
            {
                x_prop[0] = x_k[0] + u_k[0]*cos(x_k[2])*h;
                x_prop[1] = x_k[1] + u_k[0]*sin(x_k[2])*h;
                x_prop[2] = x_k[2] + u_k[1]*h;
                x_prop[3] = 0;

                x_k = x_prop;

                //Rendering
                updte_pos_obj(x_k);

                draw_obj();
                
                glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
                
                FsSwapBuffers();
                // FsSleep(1);
            }
            ++idx; 
        }
    }
    bool KRRT::save_results(char* file_path)
    {
        //Create Loop

        // std::fstream myFile(file_path);
        // myFile << "Configuration#" << trials << "\n";
        // for(int i = 0; i < succ_trial; ++i)
        // {
        //     printf("%.4f,",res[i].time);
        //     myFile << res[i].time << ",";
        // }
        // printf("\n");
        // myFile << "\n";
        // for(int i = 0; i < succ_trial; ++i)
        // {
        //     printf("%.4f,",res[i].node_expansions);
        //     myFile << res[i].node_expansions << ",";
        // }
        // myFile <<"\n";
        // printf("\n");
        // for(int i = 0; i < succ_trial; ++i)
        // {
        //     printf("%.4f,",res[i].cost);
        //     myFile << res[i].cost << ",";
        // }
        // printf("\n");
        // myFile <<"\n";

        myFile.close();
    }



#endif 