#pragma once
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <KDtree.h>
#include <map.h>
#include <node.h>

#include "Dynamics.h"
#include "Sampler.h"
#include "Renderer.h"

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
    // ── Sub-systems ──────────────────────────────────────────────────────────
    Dynamics dynamics;
    Sampler  sampler;
    Renderer renderer;

    // ── Execution state used by the render loop in planner.cpp ───────────────
    Xstate       x_p;       // current robot state during plan playback
    Ustate       u_k;       // current control during plan playback
    int          idx = 0;   // current waypoint index
    std::fstream myfile;    // results CSV output stream

    // ── Plan output ───────────────────────────────────────────────────────────
    std::vector<node*> plan;

    // ── Lifecycle ─────────────────────────────────────────────────────────────
    KRRT();
    ~KRRT();

    // ── Primary interface ─────────────────────────────────────────────────────
    void LoadMap(const std::string& filepath);
    bool one_shot_plan();
    bool plan_trials();

    // ── Delegation helpers for planner.cpp render loop ───────────────────────
    void   set_objects();
    bool   ResetPos();
    void   updte_pos_obj(const Xstate& x);
    void   draw_obj();
    int    sec2msec(double sec) const;
    Xstate rk4step(const Xstate& x, const Ustate& u, double h) const;
    Xstate propagate_one_step(const Xstate& x, const Ustate& u) const;

private:
    // ── Map ───────────────────────────────────────────────────────────────────
    map map_1;

    // ── Planning configuration ────────────────────────────────────────────────
    // PI is available as a macro from object.h via the include chain.
    static constexpr double time2exit     = 20.0;
    static constexpr double tether_length = 70.0;
    static constexpr double tolerance     = 5.0;
    static constexpr int    K             = 200000;
    static constexpr int    n_scenarios   = 5;
    static constexpr int    n_trials      = 10;

    double weights[4]       = {1.0, 1.0, 1.0, 1.0};
    int    coords_start[2]  = {40, 46};
    int    coords_goal[2]   = {45, 10};
    int    block_width[2]   = {15, 15};

    int start_x_coord_array[5] = {30, 10,  5,  5, 40};
    int start_y_coord_array[5] = {20, 20, 35, 35, 46};
    int goal_x_coord_array[5]  = { 7, 30, 40, 48, 45};
    int goal_y_coord_array[5]  = {46, 46, 15, 50, 10};

    // ── Tree and plan state ───────────────────────────────────────────────────
    KDTree              Ktree;
    std::vector<node*>  tree;
    std::vector<results> data;

    Ustate u_start;
    Xstate x_start;
    Xstate x_goal;

    // ── Private methods ───────────────────────────────────────────────────────
    void   Initialize();
    bool   planner();
    bool   plan_to_goal();
    void   steer(const Xstate& x_near, const Xstate& x_rand,
                 Xstate& x_best, Ustate& u_best, double prob, bool near_goal);
    bool   ObstacleFree(const Xstate& x_near, const Xstate& x_rand,
                        Xstate& x_best, Ustate& u_best, double prob, bool near_goal);
    void   getPlan(node* q_last);
    void   CleanUp();

    double euclidean(const Xstate& a, const Xstate& b) const;
    double euclidean(double xf, double yf, double xi, double yi) const;
    double L2_norm(const Xstate& x) const;
    double calc_radius() const;
    double calc_angle(double xf, double yf, double xi, double yi) const;
    double calc_angle(int xf[], int xi[]) const;
    int    getPlanSize() const;
};
