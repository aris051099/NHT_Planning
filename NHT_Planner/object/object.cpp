#ifndef OBJECT    
    #define OBJECT
    #include "object.h"
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

    void object::Move(GLfloat i_x, GLfloat i_y, GLfloat i_angle,GLfloat i_beta)
    {
        this->x = i_x;
        this->y = i_y;
        x_w = i_x*block_side;
        y_w = (ofs_y-i_y)*block_side;
        this->angle = i_angle; 
        this->beta = i_beta;
        pos_idx.push_back(i_x);
        pos_idx.push_back(i_y);
    }

    void object::Draw_object()
    {
        // Draw_in_center();
        glColor3ub(this->r,this->g,this->b);

        glBegin(GL_QUADS);

        glTexCoord3f(0.0,0.0,0.0);
        glVertex3f(x_w,y_w,0); //0,0

        glTexCoord3f(1.0,0.0,0.0);
        glVertex3f(x_w+w,y_w,0); //1,0

        glTexCoord3f(1.0,1.0,0);
        glVertex3f(x_w+w,y_w+h,0); // 1,1

        glTexCoord3f(0.0,1.0,0.0);
        glVertex3f(x_w,y_w+h,0); // 0,1

        glEnd(); 

    }

    void object::Draw_object_coords(GLfloat i_x, GLfloat i_y, GLfloat i_w, GLfloat i_h)
    {
        float w_husky = 20; 
        float h_husky = 15;
        i_x =i_x*block_side;
        i_y =(ofs_y-i_y)*block_side;
        i_x = i_x + w_husky/2;
        i_y = i_y + h_husky/2;
        glColor3ub(this->r,this->g,this->b);

        glBegin(GL_QUADS);

        glTexCoord3f(0.0,0.0,0.0);
        glVertex3f(i_x,i_y,0); //0,0

        glTexCoord3f(1.0,0.0,0.0);
        glVertex3f(i_x+w,i_y,0); //1,0

        glTexCoord3f(1.0,1.0,0);
        glVertex3f(i_x+w,i_y+h,0); // 1,1

        glTexCoord3f(0.0,1.0,0.0);
        glVertex3f(i_x,i_y+h,0); // 0,1

        glEnd(); 	
    }


    void object::Draw_in_center()
    {
        glColor3ub(this->r,this->g,this->b);

        glBegin(GL_QUADS);

        glTexCoord3f(0.0,0.0,0.0);
        glVertex3f(x_w-w/2 ,y_w-h/2,0); //0,0

        glTexCoord3f(1.0,0.0,0.0);
        glVertex3f(x_w+w-w/2,y_w-h/2,0); //1,0

        glTexCoord3f(1.0,1.0,0);
        glVertex3f(x_w+w-w/2,y_w+h-h/2,0); // 1,1

        glTexCoord3f(0.0,1.0,0.0);
        glVertex3f(x_w-w/2,y_w+h-h/2,0); // 0,1

        glEnd(); 
    }

    // void object::Draw_object_Angle()
    // {
        
    // 	// if(this->angle < 0)
    // 	// {
    // 	// 	this->angle = 2*PI-this->angle;
    // 	// }
    // 	this->angle = (this->angle/PI) * 180;

    // 	// glLoadIdentity();
    // 	glPushMatrix();

    // 	glTranslatef(x_w,y_w, 0);      
    // 	glRotatef(angle, 0.0f, 0.0f, -1.0f);
    // 	glTranslatef(-x_w, -y_w, 0);

    // 	Draw_in_center();

    // 	glColor3ub(0,0,0);
    // 	glLineWidth(1);
    // 	glBegin(GL_LINES);
    // 	glVertex2f(x_w,y_w);
    // 	glVertex2f(x_w+20,y_w);
    // 	glEnd();

    // 	glPopMatrix();
    // }

    void object::Draw_object_Angle()
    {
        
        // if(this->angle < 0)
        // {
        // 	this->angle = 2*PI-this->angle;
        // }
        this->angle = (this->angle/PI) * 180;

        // glLoadIdentity();
        glPushMatrix();

        glTranslatef(x_w+w/2,y_w+h/2, 0);      
        glRotatef(angle, 0.0f, 0.0f, -1.0f);
        glTranslatef(-x_w-w/2, -y_w-h/2, 0);

        glColor3ub(this->r,this->g,this->b);

        glBegin(GL_QUADS);

        glTexCoord3f(0.0,0.0,0.0);
        glVertex3f(x_w ,y_w,0); //0,0

        glTexCoord3f(1.0,0.0,0.0);
        glVertex3f(x_w+w,y_w,0); //1,0

        glTexCoord3f(1.0,1.0,0);
        glVertex3f(x_w+w,y_w+h,0); // 1,1

        glTexCoord3f(0.0,1.0,0.0);
        glVertex3f(x_w,y_w+h,0); // 0,1
        glEnd(); 

        
        glColor3ub(0,0,0);
        glLineWidth(1);
        glBegin(GL_LINES);
        glVertex2i(x_w+w/2,y_w+h/2);
        glVertex2i(x_w+w/2+20,y_w+h/2);
        glEnd();

        glColor3ub(255,0,100);
        glLineWidth(2);
        glBegin(GL_LINES);
        glVertex2i(x_w,y_w+h/2);
        glVertex2i(x_w-20*cos(beta),y_w+h/2-20*sin(beta));
        glEnd();

        // printf("Beta(degrees):%f ; sinBeta:%f\n",(beta/PI)*180,sin(beta));


        glPopMatrix();
    }

    void object::Draw_coords()
    {
        glColor3ub(0,0,0);
        glRasterPos2f(x_w+w,y_w+2*h);
        char str[256];
        sprintf(str,"%.f,%.f",x,y);
        YsGlDrawFontBitmap12x16(str);
    }

    void object::Draw_Path()
    {
        if(pos_idx.size() > 4)
        {
            for(int j = 0; j < pos_idx.size(); j+=2)
            {
                Draw_object_coords(pos_idx[j],pos_idx[j+1],15,15);
            }
        }
    }
    void tether::set_anchor(float i_x, float i_y)
    {
        anchor_point[0] = i_x;
        anchor_point[1] = i_y;
        a_x = i_x*block_side;
        a_y = (ofs_y-i_y)*block_side;
    }
    void tether::Draw_anchor()
    {
        glColor3ub(this->r,this->g,this->b);

        glBegin(GL_QUADS);

        glTexCoord3f(0.0,0.0,0.0);
        glVertex3f(a_x,a_y,0); //0,0

        glTexCoord3f(1.0,0.0,0.0);
        glVertex3f(a_x+w,a_y,0); //1,0

        glTexCoord3f(1.0,1.0,0);
        glVertex3f(a_x+w,a_y+h,0); // 1,1

        glTexCoord3f(0.0,1.0,0.0);
        glVertex3f(a_x,a_y+h,0); // 0,1

        glEnd(); 
    }
    bool tether::point_in_line(float o_x,float o_y,float a_x,float a_y,int& AorB)
    {
        double x2 = x+20/2;
        double y2 = y+15/2;
        double x1 = a_x+w/2;
        double y1 = a_y+h/2;

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
        glVertex2i(xi+w/2,yi+h/2);
        glVertex2i(xf+20/2,yf+15/2);
        glEnd();
    }
    bool tether::contact_point(std::unordered_map<int,std::tuple<double,double>>& obs_set)
    {
        int AorB = 0;
        double obs_coord[2] = {0,0};
        for(auto iter = obs_set.begin();iter != obs_set.end();++iter)
        {
            auto cur = iter->second; 
            if(point_in_line(block_side*std::get<0>(cur),block_side*(50-std::get<1>(cur)),anchor_point[0],anchor_point[1],AorB))
            {
                if(AorB == 1)
                {
                    ct_points.push_back(obs_coord[0]);
                    ct_points.push_back(obs_coord[1]);
                }
            }
        }
        return true;
    }
    void tether::Draw_tether()
    {
        Draw_anchor();
        draw_line(x_w,y_w,a_x,a_y);
    }
#endif 