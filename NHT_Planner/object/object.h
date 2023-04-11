#include <stdlib.h>
#include <random>
#include <algorithm>
#include <string>
#include <stdexcept>
#include <regex> // For regex and split logic
#include <iostream> // cout, endl
#include <fstream> // For reading/writing files
#include <vector> 
#include <fssimplewindow.h>
#include <ysglfontdata.h>
#include <unordered_map>
#define PI 3.141592654

#define GETMAPINDEX_UL0(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

class object 
{
	public:
		GLfloat x=0,y=0,angle=0,w=0,h=0,x_w=0,y_w=0,beta = 0;
		GLubyte r=0,g=0,b=0;
		float block_side = 15.0;
		float ofs_y = 50;
		std::vector<double> pos_idx; 
		object(void);
		object(GLfloat i_x, GLfloat i_y, GLfloat i_w, GLfloat i_h);
		void setColor(GLubyte i_r, GLubyte i_g, GLubyte i_b);
		void setDim(GLfloat i_w, GLfloat i_h);
		void Move(GLfloat i_x, GLfloat i_y, GLfloat i_angle, GLfloat i_beta);
		void Draw_object(void);
		void Draw_object_coords(GLfloat i_x, GLfloat i_y, GLfloat i_w, GLfloat i_h);
		void Draw_in_center(void);
		void Draw_object_Angle(void);
		void Draw_coords(void);
		void Draw_Path(void);
};

class tether: public object 
    {
        public: 
            float anchor_point[2] = {0,0};
            float a_x =0, a_y = 0;
            std::vector<float> ct_points;
            void set_anchor(float x,float y);
            void Draw_anchor();
            bool point_in_line(float o_x,float o_y,float a_x,float a_y,int& AorB);
            bool contact_point(std::unordered_map<int,std::tuple<double,double>>& obs_set);
            void draw_line(float xf,float yf,float xi,float yi);
            void Draw_tether();
    };