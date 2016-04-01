#include <boost/python.hpp>

#include "../Sensor.h"
#include "../Vector3D.h"
#include <pthread.h>

bool terminate;
pthread_spinlock_t I2C_ACCESS;

using namespace boost::python;

BOOST_PYTHON_MODULE(Quad) {
  class_<Sensor>("Sensor", init<float>())
    .def("initialize", &Sensor::initialize)
    .def("getMotionData", &Sensor::getMotionData)
  ;

  class_<Vector3D>("Vector3D")
    .def_readonly("x", &Vector3D::x)
    .def_readonly("y", &Vector3D::y)
    .def_readonly("z", &Vector3D::z)
  ;
}
