#ifndef VECTOR2D_HPP
#define VECTOR2D_HPP

#include <iostream>

#define PI 3.14159265

enum Angle{
  DEG = 0,
  RAD = 1
};

class Vector2D{

  /*overloaded operators*/
  friend std::ostream& operator<<(std::ostream &os, const Vector2D &v);
  friend Vector2D operator+(const Vector2D &v1,const Vector2D &v2);
  friend Vector2D operator-(const Vector2D &v1,const Vector2D &v2);
  friend Vector2D operator*(double scal, const Vector2D &v);
  friend Vector2D operator/(double scal, const Vector2D &v);



public:

  Vector2D();
  Vector2D(double x, double y);
  Vector2D(const Vector2D &v);
  Vector2D(double angle, Angle a);

  double dot(const Vector2D &v) const;    ///< 2D dot product
  double cross(const Vector2D &v);  ///< 2D cross product
  double magnitute(void) const;           ///< vector length
  double degree(const Vector2D &v) const;       ///< angle in dregree between two vectors
  double rad(const Vector2D &v) const;          ///< angle in rad between two vectors
  double getX(void) const;                ///< get x value of vector
  double getY(void) const;                ///< get y value of vector
  bool isnan(void);

  Vector2D orthogonal(void);
  Vector2D& rotate(double angle, Angle a);
  Vector2D& normalize(void);       ///< normalize vector
  Vector2D& getVector(void) {return *this;}


  Vector2D& operator+=(const Vector2D &v);
  Vector2D& operator-=(const Vector2D &v);
  Vector2D& operator=(const Vector2D &v);
  Vector2D& operator*=(const double scalar);
  Vector2D& operator/=(const double scalar);
  bool operator!=(Vector2D &v2);
  bool operator==(Vector2D &v2);
  //Vector2D  operator*(double scal, Vector2D &v);

private:

  double xdir,ydir;
};

#endif /*VECTOR2_HPP*/
