#ifndef MAP    
    #define MAP
    #include "map.h"
    map::map(void) {};
    map::map(double *map_i_ptr)
    {
        this->map_ptr = map_i_ptr;
    }

    void map::setWhite()
    {
        this->r = 255;
        this->g = 255;
        this->b = 255;
    }
    void map::setBlack()
    {
        this->r = 0;
        this->g = 0;
        this->b = 0;          
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
                    map_ptr[y*width+x] = 1; // Note transposed from visual
                } 
                else 
                {
                    map_ptr[y*width+x] = 0;
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

    void map::printMap()
    {
        for(int i = 0; i < width*height ; ++i)
        {
            if(i%height == 0)
            {
                std::cout << std::endl;
            }
            std::cout << map_ptr[i] << " ";
        }
    }

    void map::calc_collision_set()
    {
        int x_e = 0;
        int y_e = 0;
        double coord[2] = {0,0};
        for(int i = 0; i < width*height ; ++i)
        {
            if(map_ptr[i] == 1.0)
            {
                obstacle_set.insert({i,std::make_tuple(x_e,y_e)});
            }
            ++x_e;
            if(x_e>width)
            {
                x_e = 0;
            }
            if(i>0 && i%height == 0)
            {
                ++y_e;
            }
        }
    }

#endif 