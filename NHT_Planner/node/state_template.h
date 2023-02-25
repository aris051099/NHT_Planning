template <class ComponentType,int NumComponent>
class state
{
  protected:
    double state_elem[NumComponent] = {};
    int arr_size = NumComponent;
  public:
    state(){}
    const int size() const
    {
      return arr_size;
    }
    const ComponentType *getPointer() const
    {
      return state_elem;
    }
    ComponentType *getEditPointer()
    {
      return state_elem;
    }
    double operator[] (int index) const
    {
      if(index > arr_size)
      {
        printf("Index out of bounds");
      }
      return state_elem[index];
    } 
    double& operator[] (int index)
    {
      if(index > arr_size)
      {
        printf("Index out of bounds");
      }
      return state_elem[index];
    } 
    //   Test& operator=(const Test& t)
    // {
    //     cout << "Assignment operator called " << endl;
    //     return *this;
    // }

};
