#ifndef KiRTT
#define KiRTT
#include "KRRT.h"

// ── Constructor / Destructor ─────────────────────────────────────────────────

KRRT::KRRT()
{
    Initialize();
}

KRRT::~KRRT()
{
    CleanUp();
    plan.clear();
}

// ── Public interface ─────────────────────────────────────────────────────────

void KRRT::LoadMap(const std::string& filepath)
{
    map_1.loadMap(filepath);
    map_1.calc_collision_set();
}

int KRRT::sec2msec(double sec) const
{
    return dynamics.sec2msec(sec);
}

Xstate KRRT::rk4step(const Xstate& x, const Ustate& u, double /*h*/) const
{
    return dynamics.rk4step(x, u);
}

Xstate KRRT::propagate_one_step(const Xstate& x, const Ustate& u) const
{
    return dynamics.euler_step(x, u);
}

void KRRT::set_objects()
{
    renderer.setup(coords_start, coords_goal, block_width);
}

void KRRT::updte_pos_obj(const Xstate& x)
{
    renderer.update(x);
}

void KRRT::draw_obj()
{
    renderer.draw(map_1);
}

bool KRRT::ResetPos()
{
    if(idx >= getPlanSize())
    {
        idx = 0;
        x_p = plan[0]->getXstate();
        renderer.reset_trail(coords_start);
        return true;
    }
    return false;
}

// ── Private helpers ──────────────────────────────────────────────────────────

void KRRT::Initialize()
{
    u_start.setState(0, 0, 0);
    x_start.setState(coords_start[0], coords_start[1],
                     calc_angle(coords_goal, coords_start), 0);
    x_goal.setState(coords_goal[0], coords_goal[1], PI / 2.0, 0);
    x_p = x_start;
}

void KRRT::CleanUp()
{
    Ktree.cleanUp();
}

void KRRT::getPlan(node* q_last)
{
    for(node* p = q_last; p != nullptr; p = plan.back()->getParent())
        plan.push_back(p);
    std::reverse(plan.begin(), plan.end());
}

int KRRT::getPlanSize() const
{
    return (int)plan.size();
}

double KRRT::euclidean(const Xstate& a, const Xstate& b) const
{
    double dist = 0;
    for(int i = 0; i < a.size(); ++i)
        dist += weights[i] * (a[i] - b[i]) * (a[i] - b[i]);
    return std::sqrt(dist);
}

double KRRT::euclidean(double xf, double yf, double xi, double yi) const
{
    return std::sqrt((xf - xi) * (xf - xi) + (yf - yi) * (yf - yi));
}

double KRRT::L2_norm(const Xstate& x) const
{
    double sum = 0;
    for(int i = 0; i < x.size(); ++i)
        sum += x[i] * x[i];
    return std::sqrt(sum);
}

double KRRT::calc_radius() const
{
    double d     = 3.0;
    double gamma = 500.0;
    double delta = (PI * PI) / 2.0;
    double V     = (double)Ktree.size;
    return (gamma / delta) * std::pow(std::log(V) / V, 1.0 / d);
}

double KRRT::calc_angle(double xf, double yf, double xi, double yi) const
{
    double angle = std::atan2(yf - yi, xf - xi);
    if(angle < 0) angle += 2.0 * PI;
    return angle;
}

double KRRT::calc_angle(int xf[], int xi[]) const
{
    double angle = std::atan2(xf[1] - xi[1], xf[0] - xi[0]);
    if(angle < 0) angle += 2.0 * PI;
    return angle;
}

// ── Steering ─────────────────────────────────────────────────────────────────

void KRRT::steer(const Xstate& x_near, const Xstate& x_rand,
                 Xstate& x_best, Ustate& u_best, double prob, bool near_goal)
{
    double min_dist = std::numeric_limits<double>::infinity();

    for(int i = 0; i < 20; ++i)
    {
        Ustate u_k = (near_goal && prob > 0.95)
            ? sampler.sample_near_goal_control()
            : sampler.sample_control();

        Xstate x_prop = dynamics.propagate(
            x_near, u_k, map_1.map_ptr, map_1.width, map_1.height);

        double dist = euclidean(x_rand, x_prop);
        if(dist < min_dist)
        {
            u_best   = u_k;
            min_dist = dist;
            x_best   = x_prop;
        }
    }
}

bool KRRT::ObstacleFree(const Xstate& x_near, const Xstate& x_rand,
                        Xstate& x_best, Ustate& u_best, double prob, bool near_goal)
{
    steer(x_near, x_rand, x_best, u_best, prob, near_goal);
    return x_best.state != 2;
}

// ── Core planner ─────────────────────────────────────────────────────────────

bool KRRT::planner()
{
    Xstate x_rand, x_best;
    Ustate u_best;

    bool   near_goal  = false;
    double accum_time = 0.0;
    double t_passed   = 0.0;
    int    quad       = 5;

    node* qstart = new node(t_passed, euclidean(x_goal, x_start),
                            nullptr, u_start, x_start);
    Ktree.Insert(qstart);

    std::cout << "Number of samples: " << K << std::endl;

    for(int i = 0; i < K; ++i)
    {
        auto   start_time = std::chrono::high_resolution_clock::now();
        double prob       = sampler.sample_prob();

        if(prob > 0.95)
        {
            x_rand = x_goal;
        }
        else if(prob > 0.85)
        {
            x_rand = sampler.sample_near_goal_state(x_goal[0], x_goal[1], quad);
        }
        else
        {
            x_rand = sampler.sample_random_state();
        }

        if(x_rand[0] > map_1.width && x_rand[1] > map_1.height)
            continue;

        double r      = L2_norm(x_rand);
        node*  q_near = Ktree.nearest_neighbor(x_rand, r);
        if(q_near == nullptr)
            continue;

        if(ObstacleFree(q_near->getXstate(), x_rand, x_best, u_best, prob, near_goal))
        {
            near_goal = false;

            node* q_new = new node(
                q_near->g + u_best[0] * u_best.get_tprop(),
                euclidean(x_goal, x_best),
                q_near, u_best, x_best);
            Ktree.Insert(q_new);

            double dist2goal = euclidean(x_goal, q_new->getXstate());

            if(dist2goal < tolerance && q_new->g < tether_length)
            {
                printf("Goal found at K: %d\n", i);
                getPlan(q_new);
                return true;
            }
            else if(dist2goal < tolerance + 2.0)
            {
                near_goal = true;
            }

            auto end_time   = std::chrono::high_resolution_clock::now();
            auto time_delay = std::chrono::duration_cast<std::chrono::microseconds>(
                end_time - start_time);
            accum_time += time_delay.count() * 1e-6;

            if(accum_time > time2exit)
                return false;
        }
    }

    printf("No plan found\n");
    return false;
}

bool KRRT::plan_to_goal()
{
    bool too_long = false;
    int  n_tries  = 0;

    while(!too_long)
    {
        CleanUp();
        plan.clear();

        if(planner())
        {
            too_long = true;
        }
        else
        {
            ++n_tries;
            printf("Number of tries: %d\n", n_tries);
            if(n_tries >= 50)
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
        auto end_time   = std::chrono::system_clock::now();
        auto time_delay = std::chrono::duration_cast<std::chrono::nanoseconds>(
            end_time - start_time);
        std::cout << "--------------- RESULTS ---------------" << std::endl;
        std::cout << "Planning time: " << time_delay.count() * 1e-9 << " seconds" << std::endl;
        std::cout << "Tree size:     " << Ktree.size                 << std::endl;
        std::cout << "Cost of plan:  " << plan.back()->g             << std::endl;
        std::cout << "---------------------------------------" << std::endl;
        return true;
    }
    return false;
}

bool KRRT::plan_trials()
{
    myfile.open("C:/Users/arisa/Desktop/Path_Planning/NHT_Planning/NHT_Planner/results.csv");
    for(int trial = 0; trial < n_scenarios; ++trial)
    {
        results res[n_trials];
        int succ_trial = 0;

        coords_start[0] = start_x_coord_array[trial];
        coords_start[1] = start_y_coord_array[trial];
        coords_goal[0]  = goal_x_coord_array[trial];
        coords_goal[1]  = goal_y_coord_array[trial];

        x_start.setState(coords_start[0], coords_start[1],
                         calc_angle(coords_goal, coords_start), 0);
        x_goal.setState(coords_goal[0], coords_goal[1], PI / 2.0, 0);

        printf("Start: %.2f, %.2f, %.2f\n", x_start[0], x_start[1], x_start[2]);
        printf("Goal:  %.2f, %.2f, %.2f\n", x_goal[0],  x_goal[1],  x_goal[2]);

        while(succ_trial < n_trials)
        {
            auto start_time = std::chrono::system_clock::now();
            if(plan_to_goal())
            {
                auto end_time   = std::chrono::system_clock::now();
                auto time_delay = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    end_time - start_time);
                res[succ_trial].time            = time_delay.count() * 1e-9;
                res[succ_trial].cost            = plan.back()->g;
                res[succ_trial].node_expansions = Ktree.size;
                ++succ_trial;
            }
            else
            {
                return false;
            }
        }

        for(int i = 0; i < succ_trial; ++i)
            myfile << res[i].time << "," << res[i].node_expansions << "," << res[i].cost << "\n";
    }
    myfile.close();
    return true;
}

#endif
