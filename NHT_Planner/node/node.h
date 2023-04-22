#pragma once
#include <iostream> 
#include <math.h>
#include <algorithm>
#include <chrono>
#include <stdlib.h>
#include <vector>
#include <Xstate.h>

class node 
{
	public:
		node* parent = nullptr;
		Ustate u_control; 
		Xstate reached_state;
		double f=0,g=0,h=0; 
		int axis = -1;
		node* left = nullptr;
		node* right = nullptr;
		std::vector<node*> childs;
		node(){};
		node(const Xstate&p,int axis,node* parent): reached_state{p},axis{axis},parent{parent} {};
		node(double g,double h, node* inc_parent,const Ustate& inc_u_control,const Xstate& X_inc):g{g},h{h},parent{inc_parent},reached_state{X_inc}
		{
			f = g+h;
			reached_state = X_inc;
			u_control = inc_u_control;
		}
		void setParent(node* incoming);
		node *getParent();
		const Xstate& getconstXstate() const;
		Xstate& getXstate();
		Ustate& getUstate();
		void setXstate(Xstate& x);
		void setUstate(Ustate& u);
		void calc_f(double inc_g, double inc_h);
		void calc_f();
		void update(node* parent,Xstate new_x,Ustate new_u,double new_g);
};