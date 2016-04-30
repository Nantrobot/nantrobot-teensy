#ifndef FIR_H
#define FIR_H
#ifndef Pi
#define Pi 3.1415927
#endif

#include<math.h>
#include"ringbuff.h"
#include"Arduino.h"

template<int n> class bandpass_fir{
  
  private:
    float h[n];
    float fs,ft1,ft2;
    int M=n-1;
    
  public:
    bandpass_fir(void);
    virtual void initialize(float f1,float f2,float fe,const char * window);
    float compute(ring_buffer<n> & x);

};

template<int n> class lowpass_fir: public bandpass_fir<n>{
  public:
    void initialize(float fc,float fe,const char * window){
      bandpass_fir<n>::initialize(0,fc,fe,window);
      };
};



template<int n> bandpass_fir<n>::bandpass_fir(void):h(),fs(0),ft1(0),ft2(0){
}

template<int n> void bandpass_fir<n>::initialize(float f1,float f2,float fe,const char * window){
  //Normalize frequency
  fs=fe;
  ft1=f1/fe;
  ft2=f2/fe;
  float M_2=M/2;
  for(int i=0;i<n;i++){
    if(i==M_2){
      h[i]=2*(ft2-ft1);
    }else{
      h[i]=(sin(2*Pi*ft2*((float)i-M_2))-sin(2*Pi*ft1*((float)i-M_2)))/(Pi*((float)i-M_2));
    }
  }
  if(strcmp(window,"hanning")==0){
    for(int i=0;i<n;i++){
      h[i]=h[i]*(0.5-0.5*cos(2*Pi*(float)i/M));
    }
  }else if(strcmp(window,"hamming")==0){
     for(int i=0;i<n;i++){
      h[i]=h[i]*(0.54-0.46*cos(2*Pi*(float)i/M));
    }
  }else if(strcmp(window,"blackman")==0){
     for(int i=0;i<n;i++){
      h[i]=h[i]*(0.42-0.5*cos(2*Pi*(float)i/M)+0.08*cos(4*Pi*(float)i/M));
    }
  }
}


template<int n> float bandpass_fir<n>::compute(ring_buffer<n> &x){
  float output=0;
  for(int i=0;i<n;i++){
    output+=(x[i])*h[i];
  }
  return(output);
}





#endif
