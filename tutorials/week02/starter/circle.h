#ifndef CIRCLE_H
#define CIRCLE_H


class Circle
{
public:
  Circle(float x);
  void setRadius(float x);
  float getArea(void);
  float getPerimeter(void);
  void setArea(float x);
  void setPerimeter(float x);
  float print(void);

private:
  float radius_;

};

#endif // CIRCLE_H
