void print_angles(vertice* a)
{
	for(int i = 0; i < a->getDOF();++i)
	{
		cout<< a->angles[i] << ", "; 
	}
	cout << endl;
}
void random_state(double*& angles, int DOFs)
{
	for(int i = 0;i < DOFs ;++i)
	{
		angles[i] = (double) rand_sample();
	}
}
double euclidean(vertice*& a,vertice*& b)
{
	double accum = 0;
	int dof = a->getDOF();
	for(int i = 0;i< dof;++i)
	{
		accum +=(a->angles[i] - b->angles[i])*(a->angles[i]- b->angles[i]);	
	}
	return sqrt(accum);
}
vertice* normalized_direc(vertice*& a, vertice*& b,int DOF)
{
	vertice* c = new vertice(DOF);
	double norm = euclidean(a,b);
	double ang_t;
	for(int i = 0; i < DOF;++i)
	{
		c->angles[i] = (b->angles[i] - a->angles[i])/norm; 
		ang_t = c->angles[i];
	}
	return c;
} 
double getRadius(vertice*& random_nod)
{
	double accum = 0;
	int size = random_nod->getDOF();
	for(int i = 0;i< size ;++i)
	{
		accum +=random_nod->angles[i]*random_nod->angles[i];	
	}
	return sqrt(accum);
};
