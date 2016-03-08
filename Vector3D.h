//Vector Helper Class

#ifndef _VECTOR_3D_
#define _VECTOR_3D_

class Vector3D {
 public:
  Vector3D ();
  Vector3D (float tx, float ty, float tz);
  Vector3D (const Vector3D &t);
  Vector3D& operator= (const Vector3D &t);
  Vector3D operator- ();
  Vector3D operator+ (const Vector3D &t);
  Vector3D operator- (const Vector3D &t);
  friend Vector3D operator* (const float &a, const Vector3D &t);

  void print();
  
  void setValue (float tx, float ty, float tz);
  void scale (float factor);
  float innerProduct (Vector3D &t);
  Vector3D crossProduct (Vector3D &t);
  float norm ();
  void normalize();
  float getAngle (Vector3D &t);
  void rotate (float theta, Vector3D axis);
  void rotate_unit_axis (float theta, Vector3D &axis);

  float x, y, z;
};

#endif


