#include "Vector3D.h"
#include <math.h>
#include <stdio.h>

Vector3D::Vector3D () {
  x = 0;
  y = 0;
  z = 0;
}

Vector3D::Vector3D (float tx,  float ty, float tz) {
  x = tx;
  y = ty;
  z = tz;
}

Vector3D::Vector3D (const Vector3D &t) {
  x = t.x;
  y = t.y;
  z = t.z;
}

Vector3D& Vector3D::operator= (const Vector3D &t) {
  x = t.x;
  y = t.y;
  z = t.z;
  return *this;
}

Vector3D Vector3D::operator- () {
  return Vector3D (-x, -y, -z);
}

Vector3D Vector3D::operator+ (const Vector3D &t) {
  return Vector3D (x+t.x, y+t.y, z+t.z);
}

Vector3D Vector3D::operator- (const Vector3D &t) {
  return Vector3D (x-t.x, y-t.y, z-t.z);
}

Vector3D operator* (const float &a, const Vector3D &t) {
  return Vector3D (a*t.x, a*t.y, a*t.z);
}

void Vector3D::print () {
  printf ("(%f, %f, %f)", x, y, z);
}

void Vector3D::setValue (float tx, float ty, float tz) {
  x = tx;
  y = ty;
  z = tz;
}

void Vector3D::scale (float factor) {
  x *= factor;
  y *= factor;
  z *= factor;
}

float Vector3D::innerProduct (Vector3D &t) {
  return (x* t.x + y * t.y + z * t.z);
}

Vector3D Vector3D::crossProduct (Vector3D &t) {
  return Vector3D (y*t.z - z*t.y, z*t.z - x*t.z, x*t.y - y*t.x);
}

float Vector3D::norm () {
  return sqrt(x*x + y*y + z*z);
}

void Vector3D:: normalize() {
  float n = sqrt(this->norm());
  if (n != 0) {
    x /= n;
    y /= n;
    z /= n;
  }
}

float Vector3D::getAngle (Vector3D &t) {
  float cosine = this->innerProduct(t) / this->norm() / t.norm();
  return acos(cosine);
}

void Vector3D::rotate (float theta, Vector3D axis) {
  axis.normalize();
  float vdotw = this->innerProduct(axis);
  float s = sin(theta); float c = cos(theta);
  x = vdotw*axis.x + c*(x - vdotw*axis.x) + s*(axis.y*z - axis.z*y);
  y = vdotw*axis.y + c*(y - vdotw*axis.y) + s*(axis.z*x - axis.x*z);
  z = vdotw*axis.z + c*(z - vdotw*axis.z) + s*(axis.x*y - axis.y*x);
}

void Vector3D::rotate_unit_axis (float theta, Vector3D& axis) {
  float vdotw = this->innerProduct(axis);
  float s = sin(theta); float c = cos(theta);
  x = vdotw*axis.x + c*(x - vdotw*axis.x) + s*(axis.y*z - axis.z*y);
  y = vdotw*axis.y + c*(y - vdotw*axis.y) + s*(axis.z*x - axis.x*z);
  z = vdotw*axis.z + c*(z - vdotw*axis.z) + s*(axis.x*y - axis.y*x);
}
