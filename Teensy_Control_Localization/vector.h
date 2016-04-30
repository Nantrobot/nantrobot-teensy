#ifndef VECTOR_H
#define VECTOR_H

#ifndef NULL
#define NULL 0
#endif
//Xavier DAUPTAIN
//Dernière modification : 01/03/2016 .
//This is a std::vector-like implementation for linked lists in a microncontroller.Beware: memory allocated from the heap !

template<typename T> class Node {
  private:
    T value;
    int count;
    Node<T> * follow;
  public:
    Node(void);
    ~Node(void);
  template<typename H> friend class vector;
};

template<typename T> Node<T>::Node(void):value(),count(1),follow(NULL){
};

template<typename T> Node<T>::~Node(void){
  if(follow!=NULL){
    delete(follow);
  }
}


template<typename T> class vector{
    public: 
            Node<T> * head;
            vector(void);
            ~vector(void);
            T pull_back(void);
            void push_back(T new_val);
            T pull(void);
            void push(T new_val);
            T operator [] (int index);
            bool isEmpty(void);
            int getCount(void);
            void clear(void);
    private:vector(const vector<T> &);
            vector<T> & operator = (const vector<T> &);
};

template<typename T> vector<T>::vector(void):head(NULL){
}

template<typename T> vector<T>::~vector(void){
  delete(head);
}

template<typename T> void vector<T>::clear(void){
  delete(head);
}

template<typename T> bool vector<T>::isEmpty(void){
  return(head==NULL? true:false);
}

template<typename T> int vector<T>::getCount(void){
  return(head==NULL? 0:(head->count));
}


template<typename T> void vector<T>::push_back(T new_val){
  if(head==NULL){
    head= new Node<T>;
    head->value=new_val;
    (head->count)=1;
  }else{
    Node<T> * temp=head;
    while(temp->follow!=NULL){
      (temp->count)++;
      temp=temp->follow;
    }
    (temp->count)++;
    temp->follow =new Node<T>;
    temp=temp->follow;
    temp->value=new_val;
    (temp->count)=1;
  }  
}

template<typename T>  T vector<T>::pull_back(void){
  if(head==NULL){
    T fake;
    return(fake);
  }else if(head->follow==NULL){
     T val= head->value;
     delete(head);
     head=NULL;
     return(val);
  }else{
    Node<T> * temp_prec;
    Node<T> * temp=head;
    while(temp->follow!=NULL){
     (temp->count)--;
     temp_prec=temp;
     temp=temp->follow;
    }
    T val= temp->value;
    delete(temp);
    (temp_prec->follow)=NULL;
    return(val);
  }
}


template<typename T>  T vector<T>::operator [](int index){
  if(head==NULL || index>=head->count || index<0 ){
    T fake;
    return(fake);
  }else{
    Node<T> * temp=head;
    while ((head->count)-(temp->count)!=index){
      temp=temp->follow;
    }
    return(temp->value);
  }
}



template<typename T>  T vector<T>::pull(void){
  Node<T> * temp=head;
  head=head->follow;
  temp->follow=NULL;
  T val=temp->value;
  delete(temp);
  return(val);
}

template<typename T> void vector<T>::push(T new_val){
  Node<T> * temp=new Node<T>;
  temp->value=new_val;
  temp->count=(head->count)+1;
  temp->follow=head;
  head=temp;
}
  
  
 

#endif
