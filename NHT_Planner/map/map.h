#include <stdlib.h>
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
        double *map_ptr = nullptr;
        int height = 0,width = 0; 
        int block_x = 15;
        int block_y = 15; 
        GLubyte r=0,g=0,b=0;
        std::unordered_map<int,std::tuple<double,double>> obstacle_set;
        map(void);
        map(double *map_i_ptr);
        void loadMap(std::string filepath);
        void setWhite();
        void setBlack();
        void renderMap();
        void printMap();
        void calc_collision_set();
};