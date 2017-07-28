#ifndef TAGPOINT_H
#define TAGPOINT_H

struct TagPoint {
  int id;
  float x;
  float y;
  float z;
  float theta;

  TagPoint(): 
    id(0),
    x(0),
    y(0),
    z(0),
    theta(0)
  {
  }

  TagPoint(int id_i,
           float x_i,
           float y_i,
           float z_i,
           float theta_i):
            
           x(x_i),
           y(y_i),
           z(z_i),
           theta(theta_i)
  {
  }
};

#endif // TAGPOINT_H
