#pragma once
#include <stdlib.h>
#include <memory>
#include <vector>
#include <tuple>
#include <random>
#include <algorithm>
#include <string>
#include <stdexcept>
#include <regex> // For regex and split logic
#include <iostream> // cout, endl
#include <fstream> // For reading/writing files
#include <fssimplewindow.h>
#include <unordered_map>

#define GETMAPINDEX_LL0(X, Y, XSIZE, YSIZE) ((YSIZE-Y-1)*XSIZE + X)
#define GETMAPINDEX_UL1(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)
#define GETMAPINDEX_UL0(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

class map 
{
    public: 
        std::unique_ptr<double[]> map_ptr;
        int height = 0,width = 0; 
        int block_x = 15;
        int block_y = 15; 
        GLubyte r=0,g=0,b=0;
        std::unordered_map<int,std::tuple<double,double>> obstacle_set;
        map(void);
        void loadMap(std::string filepath);
        void renderMap();
        void calc_collision_set();
};