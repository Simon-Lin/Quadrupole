#include <boost/python.hpp>
#include "../Sensor.h"
#include <pthread.h>

using namespace boost::python;

struct V3f {
  float x, y, z;
};
  
void getDataWrapper (Sensor &self, V3f &a, V3f &w, V3f &g) {
  self.getMotionData(a.x, a.y, a.z, w.x, w.y, w.z, g.x, g.y, g.z);
}

BOOST_PYTHON_MODULE(Quad) {
  class_<V3f>("V3f")
    .def_readonly("x", &V3f::x)
    .def_readonly("y", &V3f::y)
    .def_readonly("z", &V3f::z)
  ;
  
  class_<Sensor>("Sensor", init<float>())
    .def("initialize", &Sensor::initialize)  
  ;

  def("getDataWrapper", getDataWrapper);
}
