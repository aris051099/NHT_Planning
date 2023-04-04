#include <iostream> 
#include <math.h>
#include <algorithm>
#include <chrono>
#include <stdlib.h>
#include <vector>
#include <Xstate.h>

class node 
{
	protected:
		node* parent = nullptr;
		Ustate u_control; 
		Xstate reached_state;
	public:
		double f=0,g=0,h=0; 
		std::vector<node*> childs;
		node(){};
		node(double g,double h, node* inc_parent,Ustate inc_u_control, Xstate X_inc):g{g},h{h},parent{inc_parent},reached_state{X_inc}
		{
			f = g+h;
			reached_state = X_inc;
			u_control = inc_u_control;
		}
		void setParent(node* incoming)
		{
			parent = incoming; 
		}
		node *getParent()
		{
			return parent;
		}
		Xstate& getXstate()
		{
			return reached_state;
		}
		Ustate& getUstate()
		{
			return u_control;
		}
		void setXstate(Xstate& x)
		{
			reached_state = x;
		}
		void setUstate(Ustate& u)
		{
			u_control = u;
		}
		void calc_f(double inc_g, double inc_h)
		{
			this->g = inc_g;
			this->h = inc_h;
			this->f = inc_g + inc_h;
		}
		void calc_f()
		{
			this->f = this->g + this-> h;
		}
		void update(node* parent,Xstate new_x,Ustate new_u,double new_g)
		{
			this->setParent(parent);
			this->reached_state = new_x;
			this->u_control = new_u;
			this->g = new_g;
		}
};