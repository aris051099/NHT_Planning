#include <stdlib.h>
#include <random>
#include <algorithm>
#include <string>
#include <stdexcept>
#include <regex> // For regex and split logic
#include <iostream> // cout, endl
#include <fstream> // For reading/writing files
#include <fssimplewindow.h>

#define GETMAPINDEX_LL0(X, Y, XSIZE, YSIZE) ((YSIZE-Y-1)*XSIZE + X)
#define GETMAPINDEX_UL1(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)
#define GETMAPINDEX_UL0(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

class map 
{
    public: 
        double *map_ptr = nullptr;
        int height = 0,width = 0; 
        int block_x = 20;
        int block_y = 20; 
        GLubyte r=0,g=0,b=0;
        void loadMap(std::string filepath);
        void setWhite()
        {
            this->r = 255;
            this->g = 255;
            this->b = 255;
        }
        void setBlack()
        {
            this->r = 0;
            this->g = 0;
            this->b = 0;          
        }
        map(void);
        map(double *map_i_ptr);
        void renderMap();
    private:
    void Draw_block(int i_x, int i_y, int i_w, int i_h, int idx)
    {
        // int idx = GETMAPINDEX_UL1(i_x/20,i_y/20,this->width,this->height);
        if(this->map_ptr[idx] == 1.0)
        {
            printf("Black");
            setBlack();
            glColor3ub(this->r,this->g,this->b);
        }

        glBegin(GL_QUADS);

        glTexCoord3f(0.0,0.0,0.0);
        glVertex3f(i_x,i_y,0); //0,0

        glTexCoord3f(1.0,0.0,0.0);
        glVertex3f(i_x+i_w,i_y,0); //1,0

        glTexCoord3f(1.0,1.0,0);
        glVertex3f(i_x+i_w,i_y+i_h,0); // 1,1

        glTexCoord3f(0.0,1.0,0.0);
        glVertex3f(i_x,i_y+i_h,0); // 0,1

        glEnd(); 
    }
};

map::map(void) {};
map::map(double *map_i_ptr)
{
    this->map_ptr = map_i_ptr;
}

void map::loadMap(std::string filepath) {
	std::FILE *f = fopen(filepath.c_str(), "r");
	if (f) {
	}
	else {
		printf("Opening file failed! \n");
		throw std::runtime_error("Opening map file failed!");
	}
	if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2) {
		throw std::runtime_error("Invalid loadMap parsing map metadata");
	}
	////// Go through file and add to m_occupancy
	map_ptr = new double[height*width];

	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			char c;
			do {
				if (fscanf(f, "%c", &c) != 1) {
					throw std::runtime_error("Invalid parsing individual map data");
				}
			} while (isspace(c));

			if (!(c == '0')) 
			{
				// printf("x:%d,y:%d\n",x,y); 
				map_ptr[y+x*width] = 1; // Note transposed from visual
			} 
			else 
			{
				map_ptr[y+x*width] = 0;
			}
		}
	}
	fclose(f);
};

void map::renderMap()
{
    float i_x =0;
    float i_y =20;
    float i_w = this->width;
    float i_h = this->height;
    for(int i = 0; i < this->width*this->height ; ++i)
    {
        if(this->map_ptr[i] == 1.0)
        {
            // printf("Black");
            glColor3ub(0,0,0);
        }
        else
        {
            glColor3ub(255,255,255);
        }

        glBegin(GL_QUADS);

        glTexCoord3f(0.0,0.0,0.0);
        glVertex3i(i_x,i_y,0); //0,0

        glTexCoord3i(1.0,0.0,0.0);
        glVertex3f(i_x+i_w,i_y,0); //1,0

        glTexCoord3f(1.0,1.0,0);
        glVertex3f(i_x+i_w,i_y+i_h,0); // 1,1

        glTexCoord3f(0.0,1.0,0.0);
        glVertex3f(i_x,i_y+i_h,0); // 0,1

        glEnd(); 

        i_x+=this->block_x;
        if(i_x >= this->block_x*this->width)
        {
            i_x = 0;
            i_y +=this->block_y;
        }
    }        
}