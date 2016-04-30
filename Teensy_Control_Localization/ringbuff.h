#ifndef RINGBUFF_H
#define RINGBUFF_H


template<int n> class ring_buffer{
  
  private:
    int buffer[n];
    int index;
  public:
    ring_buffer(void);
    void push(int value);
    void clear();
    int & operator [] (int i);
    int get(int i);
    float sliding_mean();
};

template<int n> ring_buffer<n>::ring_buffer(void):buffer(),index(0){
  for(int i=0;i<n;i++){
    buffer[i]=0;
  }
}

template<int n> void ring_buffer<n>::clear(void){
  for(int i=0;i<n;i++){
    buffer[i]=0;
  }
}


template<int n> void ring_buffer<n>::push(int value){
  index++;
  if(index>=n){
    index=0;
  }
  buffer[index]=value;
}



template<int n> int & ring_buffer<n>::operator [](int i){
  int j=(index-i);
  if(j<0){
    j=n+j;
  }
  return(buffer[j]);
}


template<int n> int ring_buffer<n>::get(int i){
  int j=(index-i);
  if(j<0){
    j=n+j;
  }
  return(buffer[j]);
}


template<int n> float ring_buffer<n>::sliding_mean(){
  int sum=0;
  for(int i=0;i<n;i++){
    sum+=buffer[i];
  }
  return((float)sum/n);
}

#endif
