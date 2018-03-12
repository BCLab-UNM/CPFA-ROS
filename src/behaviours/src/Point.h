#ifndef POINT_H
#define POINT_H


struct Point {
  float x;
  float y;
  float theta;
  
  Point(): x(0), y(0) {}

  Point(float x_i,
        float y_i,
        float theta_i):

        x(x_i),
        y(y_i),
        theta(y_i)
  {
  }

};

#endif // POINT_H
