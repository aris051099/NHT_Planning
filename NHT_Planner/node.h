#include "state_template.h"
#include "dynamics.h"
class node 
{
	protected:
		node* parent = nullptr;
		Ustate u_control; 
		Xstate reached_state;

	public:
		double f=0,g=0,h=0; 
		node(){};
		node(double g,double h, node* inc_parent,Ustate& inc_u_control):g{g},h{h},parent{inc_parent}
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
		friend class nodeHdle;
};

class nodeHdle
{
	public:
		node* node_p = nullptr;

		nodeHdle(node* incoming)
		{
			node_p = incoming;
		}

		nodeHdle()
		{
			node_p = new node; 
		}
		~nodeHdle()
		{
			if(node_p != nullptr)
			{
				delete node_p;
				node_p = nullptr;
			}
		}
};