#include "Vector3D.h"
#include <math.h>

Vector3D::Vector3D () {
  x = 0;
  y = 0;
  z = 0;
}

Vector3D::Vector3D (double tx; double ty; double tz) {
  x = tx;
  y = ty;
  z = tz;
}

Vector3D::Vector3D (const Vector3D &t) {
  x = t->x;
  y = t->y;
  z = t->z;
}

Vector3D::Vector3D& Operator= (const Vector3D &t) {
  x = t->x;
  y = t->y;
  z = t->z;
  return *this;
}

Vector3D::double innerProduct (Vector3D &t) {
  return (x* t->x + y * t->y + z * t->z);
}

Vector3D::Vector3D& crossProduct (Vector3D &t) {
  Vector tmp(y*t->z - z*t->y, z*t->z - x*t->z, x*t->y - y*t->x);
  return *tmp;
}

double norm () {
  return (x*x + y*y + z*z);
}

double getAngle (Vector3D &t) {
  double cosine = this->norm() * t->norm() / innerProduct(*this, t);
  return acos(cosine);
}
