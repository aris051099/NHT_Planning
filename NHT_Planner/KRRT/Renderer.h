#pragma once
#include <object.h>
#include <map.h>
#include <Xstate.h>

// Owns all visual objects and rendering calls.
// Completely isolated from planning logic.
class Renderer
{
public:
    object husky_robot;
    object start_pos;
    object goal_pos;
    object path;
    tether t;

    // Initialise all visual objects from planning configuration.
    void setup(int coords_start[2], int coords_goal[2], int block_width[2])
    {
        start_pos.setDim(block_width[0], block_width[1]);
        start_pos.setColor(255, 0, 0);
        start_pos.Move(coords_start[0], coords_start[1], 0, 0);

        goal_pos.setDim(block_width[0], block_width[1]);
        goal_pos.setColor(255, 0, 0);
        goal_pos.Move(coords_goal[0], coords_goal[1], 0, 0);

        husky_robot.setDim(20, 15);
        husky_robot.setColor(255, 240, 10);
        husky_robot.Move(coords_start[0], coords_start[1], 0, 0);

        path.setDim(1, 1);
        path.setColor(0, 140, 255);
        path.Move(coords_start[0], coords_start[1], 0, 0);

        t.set_anchor((float)coords_start[0] + 5.0f, (float)coords_start[1]);
        t.setDim(block_width[0], block_width[1]);
        t.setColor(0, 0, 0);
    }

    // Move all visual objects to the new robot state.
    void update(const Xstate& x)
    {
        husky_robot.Move(x[0], x[1], x[2], x[3]);
        path.Move(x[0], x[1], x[2], x[3]);
        t.Move(x[0], x[1], x[2], x[3]);
    }

    // Draw everything: map, start/goal markers, robot, path trail, tether.
    void draw(map& map_1)
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

    // Reset robot to start position and clear the drawn trail.
    void reset_trail(int coords_start[2])
    {
        husky_robot.Move(coords_start[0], coords_start[1], 0, 0);
        husky_robot.pos_idx.clear();
        path.pos_idx.clear();
        t.pos_idx.clear();
    }
};
