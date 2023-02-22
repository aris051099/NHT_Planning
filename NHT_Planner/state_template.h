template <class ComponentType,int NumComponent>
class state
{
  public:
    double *state_elem = nullptr;
    int arr_size = NumComponent;
    state()
    {
      state_elem = new ComponentType[arr_size];
    }
    ~state()
    {
      if(state_elem != nullptr)
      {
        delete [] state_elem;
        state_elem = nullptr;
      }
    }
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
    double& operator[] (int index) const 
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
