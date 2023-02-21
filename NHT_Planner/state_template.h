template <class ComponentType,int NumComponent>
class state
{
  protected:
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
        delete state_elem;
        state_elem = nullptr;
      }
    }
  public:
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
};
