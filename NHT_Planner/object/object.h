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


class object 
{
	public:
		GLfloat x=0,y=0,angle=0,w=0,h=0;
		GLubyte r=0,g=0,b=0;
		std::vector<double> pos_idx; 
		int coords[2] = {0,0};
		object(void);
		object(GLfloat i_x, GLfloat i_y, GLfloat i_w, GLfloat i_h);
		void setColor(GLubyte i_r, GLubyte i_g, GLubyte i_b);
		void setDim(GLfloat i_w, GLfloat i_h);
		void Move(GLfloat i_x, GLfloat i_y, GLfloat i_angle);
		void Draw_object(void);
		void Draw_object_coords(GLfloat i_x, GLfloat i_y, GLfloat i_w, GLfloat i_h);
		void Draw_in_center(void);
		void Draw_object_Angle(void);
		void Draw_coords(void);
		void Draw_Path(void);
};

object::object(){};
object::object(GLfloat i_x, GLfloat i_y, GLfloat i_w, GLfloat i_h):
x{i_x},y{i_y},w{i_w},h{i_h}{};

void object::setColor(GLubyte i_r, GLubyte i_g, GLubyte i_b)
{
	this->r = i_r;
	this->g = i_g;
	this->b = i_b;
}
void object::setDim(GLfloat i_w, GLfloat i_h)
{
	this->h = i_h;
	this->w = i_w;
}

void object::Move(GLfloat i_x, GLfloat i_y, GLfloat i_angle)
{
	this->x = i_x;
	this->y = i_y;
	this->angle = i_angle; 
	pos_idx.push_back(i_x);
	pos_idx.push_back(i_y);
}

void object::Draw_object()
{
	glColor3ub(this->r,this->g,this->b);

	glBegin(GL_QUADS);

	glTexCoord3f(0.0,0.0,0.0);
	glVertex3f(x,y,0); //0,0

	glTexCoord3f(1.0,0.0,0.0);
	glVertex3f(x+w,y,0); //1,0

	glTexCoord3f(1.0,1.0,0);
	glVertex3f(x+w,y+h,0); // 1,1

	glTexCoord3f(0.0,1.0,0.0);
 	glVertex3f(x,y+h,0); // 0,1

	glEnd(); 

}

void object::Draw_object_coords(GLfloat i_x, GLfloat i_y, GLfloat i_w, GLfloat i_h)
{

	float w_husky = 20; 
	float h_husky = 15;
	glColor3ub(this->r,this->g,this->b);

	glBegin(GL_QUADS);

	glTexCoord3f(0.0,0.0,0.0);
	glVertex3f(i_x+w_husky/2.0,i_y+h_husky/2.0,0); //0,0

	glTexCoord3f(1.0,0.0,0.0);
	glVertex3f(i_x+i_w+w_husky/2.0,i_y+h_husky/2.0,0); //1,0

	glTexCoord3f(1.0,1.0,0);
	glVertex3f(i_x+i_w+w_husky/2.0,i_y+i_h+h_husky/2.0,0); // 1,1

	glTexCoord3f(0.0,1.0,0.0);
 	glVertex3f(i_x+w_husky/2.0,i_y+i_h+h_husky/2.0,0); // 0,1

	glEnd(); 	
}


void object::Draw_in_center()
{
	glColor3ub(this->r,this->g,this->b);

	glBegin(GL_QUADS);

	glTexCoord3f(0.0,0.0,0.0);
	glVertex3f(x-w/2 ,y-h/2,0); //0,0

	glTexCoord3f(1.0,0.0,0.0);
	glVertex3f(x+w-w/2,y-h/2,0); //1,0

	glTexCoord3f(1.0,1.0,0);
	glVertex3f(x+w-w/2,y+h-h/2,0); // 1,1

	glTexCoord3f(0.0,1.0,0.0);
 	glVertex3f(x-w/2,y+h-h/2,0); // 0,1

	glEnd(); 
}

void object::Draw_object_Angle()
{
	
	// if(this->angle < 0)
	// {
	// 	this->angle = 2*PI-this->angle;
	// }
	this->angle = (this->angle/PI) * 180;

	// glLoadIdentity();
	glPushMatrix();

	glTranslatef(x+w/2,y+h/2, 0);      
	glRotatef(angle, 0.0f, 0.0f, -1.0f);
	glTranslatef(-x-w/2, -y-h/2, 0);

	glColor3ub(this->r,this->g,this->b);

	glBegin(GL_QUADS);

	glTexCoord3f(0.0,0.0,0.0);
	glVertex3f(x ,y,0); //0,0

	glTexCoord3f(1.0,0.0,0.0);
	glVertex3f(x+w,y,0); //1,0

	glTexCoord3f(1.0,1.0,0);
	glVertex3f(x+w,y+h,0); // 1,1

	glTexCoord3f(0.0,1.0,0.0);
 	glVertex3f(x,y+h,0); // 0,1
	glEnd(); 

	
	glColor3ub(0,0,0);
	glLineWidth(1);
	glBegin(GL_LINES);
	glVertex2i(x+w/2,y+h/2);
	glVertex2i(x+w/2+20,y+h/2);
	glEnd();

	glPopMatrix();
}

void object::Draw_coords()
{
	glColor3ub(0,0,0);
	glRasterPos2i(x+w,y+2*h);
	char str[256];
	int i_x = x;
	int i_y = y;
	sprintf(str,"%d,%d",coords[0],coords[1]);
	YsGlDrawFontBitmap12x16(str);
}

void object::Draw_Path()
{
	if(pos_idx.size() > 4)
	{
		for(int j = 0; j < pos_idx.size(); j+=2)
		{
			Draw_object_coords(pos_idx[j],pos_idx[j+1],this->w,this->h);
		}
	}
}

class tether: public object 
{
    public: 
        float anchor_point[2] = {0,0};
        void set_anchor(float x,float y);
        void Draw_anchor();
        bool point_in_line(float o_x,float o_y,int& AorB);
        bool contact_point(std::unordered_map<int,double*>& obs_set);
        void draw_line(float xf,float yf,float xi,float yi);
        void Draw_tether();
};
void tether::set_anchor(float i_x, float i_y)
{
    anchor_point[0] = i_x;
    anchor_point[1] = i_y;
}
void tether::Draw_anchor()
{

    auto x = anchor_point[0];
    auto y = anchor_point[1];

	glColor3ub(this->r,this->g,this->b);

	glBegin(GL_QUADS);

	glTexCoord3f(0.0,0.0,0.0);
	glVertex3f(x,y,0); //0,0

	glTexCoord3f(1.0,0.0,0.0);
	glVertex3f(x+w,y,0); //1,0

	glTexCoord3f(1.0,1.0,0);
	glVertex3f(x+w,y+h,0); // 1,1

	glTexCoord3f(0.0,1.0,0.0);
 	glVertex3f(x,y+h,0); // 0,1

	glEnd(); 
}
bool tether::point_in_line(float o_x,float o_y, int& AorB)
{
    double x2 = x+20/2;
    double y2 = y+15/2;
    double x1 = anchor_point[0]+w/2;
    double y1 = anchor_point[1]+h/2;

    double AB = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    double AP = sqrt((o_x-x1)*(o_x-x1)+(o_y-y1)*(o_y-y1));
    double PB = sqrt((x2-o_x)*(x2-o_x)+(y2-o_y)*(y2-o_y));
    if(AB == AP+PB)
    {
        if(std::min(AP,PB) == AP)
        {
            AorB = 1; //Near A
        }
        else
        {
            AorB = 2; //Near B
        }
        return true;
    }
    else
    {
        return false;
    }
}
void tether::draw_line(float xf,float yf,float xi,float yi)
{
    glColor3ub(255,0,0);
    glLineWidth(1);
    glBegin(GL_LINES);
    glVertex2i(anchor_point[0]+w/2,anchor_point[1]+h/2);
    glVertex2i(x+20/2,y+15/2);
    glEnd();
}
bool tether::contact_point(std::unordered_map<int,double*>& obs_set)
{
    int AorB = 0;
    for(auto iter = obs_set.begin(); iter != obs_set.end();++iter)
    {
        double* obs_coord = iter->second;
        if(point_in_line(obs_coord[0],obs_coord[1],AorB))
        {
            if(AorB == 1)
            {
                pos_idx.push_back(obs_coord[0]);
                pos_idx.push_back(obs_coord[1]);
            }
        }
    }
}
void tether::Draw_tether()
{
    Draw_anchor();
    glColor3ub(255,0,0);
    glLineWidth(1);
    glBegin(GL_LINES);
    glVertex2i(anchor_point[0]+w/2,anchor_point[1]+h/2);
    glVertex2i(x+20/2,y+15/2);
    glEnd();
}