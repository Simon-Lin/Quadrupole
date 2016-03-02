//Vector Helper Class

#ifndef _VECTOR_3D_
#define _VECTOR_3D_

class Vector3D {
 public:
  Vector3D ();
  Vector3D (double tx; double ty; double tz);
  Vector3D (const Vector3D &t);
  Vector3D& Operator= (const Vector3D &t);
  
  double innerProduct (Vector3D &t);
  Vector crossProduct (Vector3D &t);
  double norm ();
  double getAngle (Vector3D &t);

  double x, y, z;
};

#endif


