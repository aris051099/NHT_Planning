#ifndef KiRTT    
    #define KiRTT
    #include "KRRT.h"

    int KRRT::sec2msec(double sec)
    {
    return (std::round(sec*100.0)/100.0)*100;
    }

    bool check_collision(Xstate x_prop, double *map, int x_size,int y_size) 
    {
    auto c_x = x_prop[0];
    auto c_y = x_prop[1];
    // double coords[2] = {std::round((x_prop[0]*cos(x_prop[3]))/0.01),std::round((x_prop[0]*sin(x_prop[3]))/0.01)};
    if(c_x> 0.0 && c_x < 1.0*x_size && c_y > 0.0 && c_y < 1.0*y_size)
    {
        // c_y = 50.0 - c_y;
        // int map_idx = (50.0-std::round(c_y))*x_size + std::round(c_x);
        int map_idx = (y_size-std::round(c_y)-1)*x_size + std::round(c_x);
        if(map[map_idx] == 1.0 )
        {
        return false;
        }
        else
        {
        return true;
        }
    }
    else
    {
        return false;
    }
    }

    Xstate KRRT::dynamics (const Xstate& x, const Ustate& u, double h)
    {
        Xstate x_prop;
        x_prop[0] = u[0]*cos(x[2]);
        x_prop[1] = u[0]*sin(x[2]);
        x_prop[2] = u[1];
        x_prop[3] = eps*u[0]*sin(x[3]) + u[1];
        return x_prop;
    }

    Xstate KRRT::rk4step(const Xstate& x, const Ustate& u, double h)
    {
        Xstate k1 = dynamics(x, u, h);
        Xstate k2 = dynamics(x + k1 * (h / 2), u, h);
        Xstate k3 = dynamics(x + k2 * (h / 2), u, h);
        Xstate k4 = dynamics(x + k3 * h, u, h);
        return x + (k1 + k2*2 + k3*2 + k4) * (h / 6);
    }
    Xstate KRRT::propagate(Xstate& i_x_k, Ustate& u_k,double *map, int x_size, int y_size)
    {
        Xstate x_prop(i_x_k); 
        Xstate x_prop2(i_x_k);
        Xstate x_k(i_x_k);
        double prop_time = u_k.get_tprop();
        // printf("Beta = %f\n",x_prop[3]);
        for(int i = 0; i < sec2msec(prop_time) ; ++i)
        {
            // x_prop[0] = x_k[0] + u_k[0]*cos(x_k[2])*h;
            // x_prop[1] = x_k[1] + u_k[0]*sin(x_k[2])*h;
            // x_prop[2] = x_k[2] + u_k[1]*h;
            // x_prop[3] = x_k[3] + eps*u_k[0]*h*sin(x_k[3]) + u_k[1]*h;
            // x_prop = propagate_one_step(x_k,u_k);
            x_prop = rk4step(x_k,u_k,h);
            // std::cout<< "x_prop = " << x_prop << std::endl;
            // std::cout << "x_prop2 = " << x_prop2 << std::endl;
            // printf("Beta = %f\n",x_prop[3]*180.0/PI);

            // if(x_prop[3] > PI/2 || x_prop[3] < -PI/2)
            // {
            //     x_k.state = 2; //Trapped 
            //     u_k.set_tprop(i/100.0);
            //     return x_k; 
            // }
            if(check_collision(x_prop,map,x_size,y_size))
            {
                x_k = x_prop;
            }
            else
            {
                x_k.state = 2; //Trapped 
                u_k.set_tprop(i/100.0);
                return x_k;
            }
        }
        x_prop.state = 1;
        return x_prop; 
    }

    Xstate KRRT::propagate_one_step( Xstate& x_k, Ustate& u_k)
    {
        Xstate x_prop(x_k);
        x_prop[0] = x_k[0] + u_k[0]*cos(x_k[2])*h;
        x_prop[1] = x_k[1] + u_k[0]*sin(x_k[2])*h;
        x_prop[2] = x_k[2] + u_k[1]*h;
        x_prop[3] = x_k[3] + eps*u_k[0]*h*sin(x_k[3]) + alpha*u_k[1]*h;
        // printf("u_k[0] = %f; u_k[1] = %f\n",u_k[0],u_k[1]);
        // printf("x_k[3] = %f; eps*u_k[0]*h*sin(x_k[3]) = %f; alpha*u_k[1]*h = %f ; x_prop[3] = %f\n",x_k[3],eps*u_k[0]*h*sin(x_k[3]),alpha*u_k[1]*h,x_prop[3]);
        return x_prop;
    }

    std::uniform_real_distribution<double> rand_t_prop(0.0,8);
    #if CONST
        std::uniform_int_distribution<int> rand_u_vel(50,95);
        std::uniform_real_distribution<double> rand_u_ang_vel(-0.25,0.25);
    #endif 

    #if !CONST
        std::uniform_real_distribution<double> rand_u_ang_vel(-PI,PI);
        std::uniform_int_distribution<int> rand_u_vel(10,95);
    #endif 


    std::uniform_int_distribution<int> near_rand_u_vel(10,50);
    std::uniform_real_distribution<double> near_rand_u_ang_vel(-2.0,2.0);
    void KRRT::Initialize()
    {
        u_start.setState(0,0,0);
        x_start.setState(coords_start[0],coords_start[1],calc_angle(coords_goal,coords_start),0);
        x_goal.setState(coords_goal[0],coords_goal[1],c_pi/2,0);
        x_p = x_start;
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
        for(int i = 0; i < x_goal.size(); i++)
        {
            dist+= weights[i]*pow(x_goal[i]-x_near[i],2);
        }
        return sqrt(dist);
    }
    double KRRT::euclidean(double xf,double yf,double xi,double yi)
    {
        return sqrt(pow(xf-xi,2)+pow(yf-yi,2));
    }
    double KRRT::L2_norm(const Xstate& x_s)
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
                        return i;
                        // if(dist < min)
                        // {
                        //     min_node_idx = i;
                        //     min = dist;	
                        // }
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
        #if LIN_TREE
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
        #endif

        #if !LIN_TREE
            Ktree.cleanUp();
        #endif
    }
    void KRRT::getPlan(std::vector<node*>& plan,node* q_last)
    {
        for(node* p = q_last; p != nullptr; p = plan.back()->getParent())
        {
            plan.push_back(p);
        }
        reverse(plan.begin(),plan.end());
    };
    void KRRT::getPlan_vector(std::vector<node*>& plan,std::vector<node*>& i_tree)
    {
        node* q_last = i_tree.back();
        for(node* p = q_last; p != nullptr; p = plan.back()->getParent())
        {
            plan.push_back(p);
        }
        reverse(plan.begin(),plan.end());   
    }
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
    void KRRT::steer(Xstate& x_near,Xstate& x_rand,map map_1,Xstate& x_best,Ustate& u_best, double prob,bool near_goal)
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
            // if(u_k[1] > alpha*u_k[1])
            // {
            //     continue;
            // }
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
    bool KRRT::ObstacleFree(Xstate& x_near,Xstate& x_rand,map map_1,Xstate& x_best,Ustate& u_best, double prob,bool near_goal)
    {
        steer(x_near,x_rand,map_1,x_best,u_best,prob,near_goal);
        if(x_best.state!=2)
        {
            return true;
        }
        else
        {
            return true;
        }
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

        std::uniform_real_distribution<double> rand_theta(-PI,PI);
        std::uniform_real_distribution<double> rand_beta(-PI/2.0,PI/2.0);

        std::uniform_int_distribution<int> rand_map_coordsx(0,999);
        std::uniform_int_distribution<int> rand_map_coordsy(0,999);

        std::uniform_real_distribution<double> dist_prob(0.0,1);

        std::uniform_int_distribution<int> n_rand_map_coordsx(x_goal[0]-quad,x_goal[0]+quad);
        std::uniform_int_distribution<int> n_rand_map_coordsy(x_goal[1]-quad,x_goal[1]+quad);

    #if LIN_TREE
        tree.push_back(new node(t_passed,euclidean(x_goal,x_start),nullptr,u_start,x_start));
    #endif
        //printing x_start
        // printf("x_start: %f,%f,%f,%f\n",x_start[0],x_start[1],x_start[2],x_start[3]);
    #if !LIN_TREE
        node* qstart = new node(t_passed,euclidean(x_goal,x_start),nullptr,u_start,x_start);
        Ktree.Insert(qstart);
    #endif
        std::cout<< "Number of samples: "<< K << std::endl; 

        for(int i = 0; i < K; ++i)
        {

            auto start_time = std::chrono::high_resolution_clock::now();
            double prob = dist_prob(gen); //Creating random number representing probability 
            if(prob > 0.95) //Goal biasing by 5% 
            {
                x_rand = x_goal; //Assigning qrandom to be goal
            } else if(prob > 0.85) //0.8
            {
                // Xstate x_r((double)n_rand_map_coordsx(gen),(double)n_rand_map_coordsy(gen),rand_theta(gen),rand_beta(gen));
                // Xstate x_r((double)n_rand_map_coordsx(gen),(double)n_rand_map_coordsy(gen),rand_theta(gen),0);
                x_rand.setState((double)n_rand_map_coordsx(gen),(double)n_rand_map_coordsy(gen),rand_theta(gen),0);
                // x_rand.setState((double)n_rand_map_coordsx(gen),(double)n_rand_map_coordsy(gen),rand_theta(gen),rand_beta(gen));
            }
            else
            {
                // Xstate x_r((double)rand_map_coordsx(gen)/10.0,(double)rand_map_coordsy(gen)/10.0,rand_theta(gen),rand_beta(gen));
                // Xstate x_r((double)rand_map_coordsx(gen)/10.0,(double)rand_map_coordsy(gen)/10.0,rand_theta(gen),0);
                x_rand.setState((double)rand_map_coordsx(gen)/10.0,(double)rand_map_coordsy(gen)/10.0,rand_theta(gen),0);
                // x_rand.setState((double)rand_map_coordsx(gen)/10.0,(double)rand_map_coordsy(gen)/10.0,rand_theta(gen),rand_beta(gen));
            }
            if(x_rand[0] > map_1.width && x_rand[1] > map_1.height)
            {
                continue;
            }

            



        #if LIN_TREE
            int nn_idx = nearest_n_idx(x_rand,tree);//Loops through the entire list for the closest neighbor

            if(nn_idx == -1)
                continue;
        #endif

        #if !LIN_TREE
            double r = L2_norm(x_rand);
            // if(tree.size() > 1)
            // {
            // 	r = std::min(L2_norm(x_rand),calc_radius(tree));
            // }
            node* q_near = Ktree.nearest_neighbor(x_rand,r); //Grabs that qnear
            if(q_near == nullptr)
                continue;
        #endif

            if(ObstacleFree(q_near->getXstate(),x_rand,map_1,x_best,u_best,prob,near_goal))
            // if(ObstacleFree(tree[nn_idx]->getXstate(),x_rand,map_1,x_best,u_best,prob,near_goal))
            {
                near_goal = false;

            #if LIN_TREE
                node* q_new = new node(tree[nn_idx]->g + u_best[0]*u_best.get_tprop(),euclidean(x_goal,x_best),tree[nn_idx],u_best,x_best);
                tree.push_back(q_new);
            #endif

            #if !LIN_TREE
                node* q_new = new node(q_near->g + u_best[0]*u_best.get_tprop(),euclidean(x_goal,x_best),q_near,u_best,x_best);
                Ktree.Insert(q_new);
            #endif

                double dist2goal = euclidean(x_goal,q_new->getXstate());

                if(dist2goal < tolerance && q_new->g < tether_length)
                {
                    printf("Goal found at K: %d\n", i);
                    // printf("Initial State: \n");
                    // std::cout << x_0 << std::endl;
                    // printf("Final state: \n");
                    // std::cout << x_min << std::endl;
                    // printf("Goal state:\n");
                    // std::cout<<x_goal<< std::endl;

                #if LIN_TREE
                    getPlan_vector(plan,tree);
                #endif    
                #if !LIN_TREE    
                    getPlan(plan,q_new);
                #endif    
                    return true;
                }
                else if(dist2goal < tolerance+2)
                {
                    near_goal = true;
                }
                auto end_time = std::chrono::high_resolution_clock::now();
                auto time_delay = std::chrono::duration_cast<std::chrono::microseconds> (end_time-start_time);
                auto time_passed = time_delay.count()*1e-6;
                accum_time+=time_passed;
                // printf("Accumulated_time: %.4fseconds\n",accum_time);
                if(accum_time > time2exit)
                {
                    return false;
                }
            }
            else
            {
                continue;
            }
            // steer(q_near->getXstate(),x_rand,map_1,x_best,u_best,prob,near_goal);
        }
        printf("No plan found \n");
        // CleanUp(tree);
        //no plan by default
        return false; 
    }
    bool KRRT::plan_trials()
    {
        myfile.open("C:/Users/arisa/Desktop/Path_Planning/NHT_Planning/NHT_Planner/results.csv");
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
                if(plan_to_goal())
                {
                    n_tries = 0;
                    too_long = false;
                    auto end_time = std::chrono::system_clock::now();
                    auto time_delay = std::chrono::duration_cast<std::chrono::nanoseconds> (end_time-start_time);
                    auto time_passed = time_delay.count()*1e-9;
                    res[succ_trial].time = time_passed;
                    res[succ_trial].cost = plan.back()->g;
                    #if LIN_TREE
                        res[succ_trial].node_expansions = tree.size();
                    #endif
                    #if !LIN_TREE
                        res[succ_trial].node_expansions = Ktree.size;
                    #endif

                    ++succ_trial;
                    // std::cout<<"--------------- RESULTS---------------"<<std::endl;
                    // std::cout<<"Planning time: "<< time_passed << " seconds" << std::endl;
                    // std::cout<<"Tree size:" << tree.size() << std::endl;
                    // std::cout<<"Cost of the plan: " << plan.back()->g << std::endl; 
                    // std::cout<<"--------------------------------------"<<std::endl;
                }
                else
                {
                    return false;
                }
            }

            // myfile << "Configuration#" << trials << "\n";
            for(int i = 0; i < succ_trial; ++i)
            {
                printf("%.4f,",res[i].time);
                myfile << res[i].time << ",";
                myfile << res[i].node_expansions << ",";
                myfile << res[i].cost << ",";
                myfile <<" \n";
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
        myfile.close();
        return true;
    }
    bool KRRT::plan_to_goal()
    {
        // seed+=rand()%500;
        // printf("Seed: %d \n",seed); 
        // gen.seed(seed); 
        bool too_long = false; 
        bool reset_planner = false;  	
        int n_tries = 0;
        while(!too_long)
        {

            if(!tree.empty())
            // if(Ktree.size > 0)
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
            // if(n_tries%3 == 0)
            // {
            //     printf("Alpha value:%f",alpha);
            //     alpha+=0.25;
            //     printf("--> %f \n",alpha);
            // }
            if(n_tries >=50)
            {
                return false;
            }
        }
        return true;	
    }
    bool KRRT::one_shot_plan()
    {
        auto start_time = std::chrono::system_clock::now();
        if(plan_to_goal())
        {
            auto end_time = std::chrono::system_clock::now();
            auto time_delay = std::chrono::duration_cast<std::chrono::nanoseconds> (end_time-start_time);
            auto time_passed = time_delay.count()*1e-9;
            std::cout<<"--------------- RESULTS---------------"<<std::endl;
            std::cout<<"Planning time: "<< time_passed << " seconds" << std::endl;
            std::cout<<"Tree size:" << Ktree.size << std::endl;
            std::cout<<"Cost of the plan: " << plan.back()->g << std::endl; 
            std::cout<<"--------------------------------------"<<std::endl;
            return true;
        }
        else
        {
            return false;
        }

    }
    void KRRT::updte_pos_obj(const Xstate& inc_x)
    {
        husky_robot.Move(inc_x[0],inc_x[1],inc_x[2],inc_x[3]);
        path.Move(inc_x[0],inc_x[1],inc_x[2],inc_x[3]);
        t.Move(inc_x[0],inc_x[1],inc_x[2],inc_x[3]);
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
    int KRRT::getPlanSize()
    {
        return plan.size();
    }
    void KRRT::Render()
    {
        int idx = 0;
        // bool end_path = false; 
        Xstate x_k(x_start);
        std::cout << "Rendering planner" << std::endl;
        // FsOpenWindow(0,0,w_width,w_height,1);
        for(;;)
        {
            FsPollDevice();
            if(FSKEY_ESC==FsInkey())
            {
                break;
            }
            if(idx >= getPlanSize())
            {	
                idx = 0;
                x_k = plan[0]->getXstate(); 
                husky_robot.Move(coords_start[0],coords_start[1],0,0);
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

                // updte_pos_obj(x_k);

                husky_robot.Move(x_prop[0],x_prop[1],x_prop[2],x_prop[3]);
                path.Move(x_prop[0],x_prop[1],x_prop[2],x_prop[3]);
                t.Move(x_prop[0],x_prop[1],x_prop[2],x_prop[3]);

                // draw_obj();

                map_1.renderMap();

                start_pos.Draw_object();
                start_pos.Draw_coords();
                
                goal_pos.Draw_object();
                goal_pos.Draw_coords();

                husky_robot.Draw_object_Angle();

                path.Draw_Path();
                t.Draw_tether();
                
                glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
                
                FsSwapBuffers();
                // FsSleep(1);
            }
            ++idx; 
        }
    }
    void KRRT::set_objects()
    {
        start_pos.setDim(block_width[0],block_width[1]);
        start_pos.setColor(255,0,0);
        start_pos.Move(coords_start[0],coords_start[1],0,0);

        goal_pos.setDim(block_width[0],block_width[1]);
        goal_pos.setColor(255,0,0);
        goal_pos.Move(coords_goal[0],coords_goal[1],0,0);

        husky_robot.setDim(20,15);
        husky_robot.setColor(255,240,10);
        husky_robot.Move(coords_start[0],coords_start[1],0,0);

        path.setDim(1,1);
        path.setColor(0,140,255);
        path.Move(coords_start[0],coords_start[1],0,0);

        t.set_anchor(coords_start[0] + 5,coords_start[1]);
        t.setDim(block_width[0],block_width[1]);
        t.setColor(0,0,0);   
    }

    bool KRRT::ResetPos()
    {
        if(idx >= getPlanSize())
            {	
                idx = 0;
                x_p = plan[0]->getXstate(); 
                husky_robot.Move(coords_start[0],coords_start[1],0,0);
                husky_robot.pos_idx.clear();
                path.pos_idx.clear();
                t.pos_idx.clear();
                return true;
            }	
        return false;
    }

#endif 