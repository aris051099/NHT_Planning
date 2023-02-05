struct vertice 
{
	vector<vertice*> childs;
	vertice* parent; //Parent for backtracking 
	double angles[6]; //DOF	
	double g,h,f;
	int eps=1;
	int state = -1;
	int id,dof;
	bool closed = false; 
    bool open = false; 
	vertice(int id,int dof):id{id},dof{dof},parent{nullptr},g{std::numeric_limits<double>::infinity()},h{0}
	{
		f = g+eps*h;
		for(int i = 0;i<dof;++i)
		{
			this->angles[i] = (double) rand_sample();
			// #if DEBUG
			// 	cout<<angles[i]<< endl;
			// #endif
		}
	};

	vertice(int dof):id{0},dof{dof},parent{nullptr},g{std::numeric_limits<double>::infinity()},h{0}
	{
		f = g+eps*h;
		for(int i = 0;i < dof;++i)
		{
			this->angles[i] = 0;
			// #if DEBUG
			// 	cout<<angles[i]<< endl;
			// #endif
		}
	};

	vertice(double* angs,int id,int dof):id{id},dof{dof},parent{nullptr},g{std::numeric_limits<double>::infinity()},h{0}
	{
		f = g+eps*h;
		for(int i = 0;i<dof ;++i)
		{
			this->angles[i] = angs[i];
			// #if DEBUG
			// 	cout<<angles[i]<< endl;
			// #endif
		}
	};

    vertice(double g,double h, double* angle,int dof)
    :g{g},h{h},parent{nullptr} 
	{
		f = g+eps*h;
		for(int i=0;i<dof;++i)
		{
			this->angles[i] = angle[i];
		}
		f = g+eps*h;
	};

	vertice(double g,double h, double* angle,vertice* father_,int id,int dof)
    :g{g},h{h},parent{father_},id{id},dof{dof} 
	{
		for(int i=0;i<dof;++i)
		{
			this->angles[i] = angle[i];
		}
		f = g+eps*h;
		this->setOpen();
	};

	vertice(double g,double h, double* angle,vertice* father_,int dof)
    :g{g},h{h},parent{father_}
	{
		for(int i=0;i<dof;++i)
		{
			this->angles[i] = angle[i];
		}
		f = g+eps*h;
		this->setOpen();
	};

	int getDOF()
	{
		return this->dof; 
	};
	  void setParent(vertice* father)
    {
        this-> parent = father;
    };
    void calcF()
    {
        f = g + eps*h;
    }
    void setClosed()
    {
        closed = true;
    }
    bool getClosed() const
    {
        return closed;
    }
    bool getOpen()const 
    {
        return open;
    }
    void setOpen()
    {
        open = true;
    }  
    void removeClosed()
    {
        closed = false;
    }
    void removeOpen()
    {
        open = false;
    }
};

struct dist_to_ver
{
	double m_dist;
	vertice* m_vert;
	dist_to_ver(double dist, vertice* vert): m_dist{dist},m_vert{vert} {};
};

struct custom
{
	bool operator() (vertice* a,vertice* b) const{
    return a->f > b->f;};
};

struct min_dist_comp
{
	bool operator() (dist_to_ver* a,dist_to_ver* b) const{
    return a->m_dist > b->m_dist;};
};

struct planner_results
{
	double time;
	int tree_size;
	int tree_b_size;
	double cost;
	bool success_less_5;
	double start_angles[5];
	double goal_angles[5];
};