#ifndef PANODUINO_PAN_STOP_MODEL_H
#define PANODUINO_PAN_STOP_MODEL_H
#include <math.h>

// Causal quadratic position fit. Unlike a window's average slope, its
// derivative describes the current instant during acceleration/braking.
// Time is relative to now; either chronological sample order is supported.
class PanStopModel
{
public:
  static bool estimate(const double *time,const double *position,unsigned int count,double &nowPosition,double &nowVelocity)
  {
    if(count<5 || count>10) return false;
    const double scale=.02,base=position[0];
    double m[3][4]={};
    for(unsigned int i=0;i<count;++i) {
      double x=time[i]/scale,basis[3]={1,x,x*x},y=position[i]-base;
      for(unsigned int row=0;row<3;++row) {
        for(unsigned int col=0;col<3;++col) m[row][col]+=basis[row]*basis[col];
        m[row][3]+=basis[row]*y;
      }
    }
    for(unsigned int col=0;col<3;++col) {
      unsigned int pivot=col;
      for(unsigned int row=col+1;row<3;++row) if(fabs(m[row][col])>fabs(m[pivot][col])) pivot=row;
      if(fabs(m[pivot][col])<1e-9) return false;
      for(unsigned int j=col;j<4;++j) { double temp=m[col][j];m[col][j]=m[pivot][j];m[pivot][j]=temp; }
      double divisor=m[col][col];for(unsigned int j=col;j<4;++j) m[col][j]/=divisor;
      for(unsigned int row=0;row<3;++row) if(row!=col) {
        double factor=m[row][col];for(unsigned int j=col;j<4;++j) m[row][j]-=factor*m[col][j];
      }
    }
    nowPosition=base+m[0][3];nowVelocity=m[1][3]/scale;return true;
  }
};
#endif
