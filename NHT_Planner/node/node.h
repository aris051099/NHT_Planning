#include <iostream> 
#include "state_template.h"
#include "dynamics.h"
#include <math.h>
#include <algorithm>
#include <chrono>
#include <stdlib.h>
class node 
{
	protected:
		node* parent = nullptr;
		Ustate u_control; 
		Xstate reached_state;

	public:
		double f=0,g=0,h=0; 
		node(){};
		node(double g,double h, node* inc_parent,Ustate inc_u_control, Xstate X_inc):g{g},h{h},parent{inc_parent},reached_state{X_inc}
		{
			f = g+h;
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
};