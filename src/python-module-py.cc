#include "dynamic-graph/python/module.hh"

#include "time.hh"
#include "delay.hh"
#include "gain-adaptive.hh"
#include "holonomic-constraint.hh"
#include "task-smith-predictor.hh"

BOOST_PYTHON_MODULE(wrap)
{
  using namespace dynamicgraph::agimus;
  bp::import("dynamic_graph");
  dg::python::exposeEntity<Time<int> >();
  dg::python::exposeEntity<SafeGainAdaptive>();
  dg::python::exposeEntity<HolonomicConstraint>();
  dg::python::exposeEntity<TaskSmithPredictor>()
    .def("initialize", &TaskSmithPredictor::initialize)
    .def("getPeriod", &TaskSmithPredictor::getPeriod)
    .def("getDelay", &TaskSmithPredictor::getDelay)
    ;
}
