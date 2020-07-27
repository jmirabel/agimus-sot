#include "task-smith-predictor.hh"

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-bind.h>

namespace dynamicgraph {
namespace cmd = command;
namespace agimus {

TaskSmithPredictor::TaskSmithPredictor(const std::string &n) : Task(n) {
  taskSOUT.setFunction(boost::bind(
      &TaskSmithPredictor::computeTaskExponentialDecrease, this, _1, _2));

  addCommand ("initialize", cmd::makeCommandVoid2(*this, &TaskSmithPredictor::initialize, ""));
  addCommand ("getPeriod", cmd::makeCommandReturnType0(*this, &TaskSmithPredictor::getPeriod, ""));
  addCommand ("getDelay", cmd::makeCommandReturnType0(*this, &TaskSmithPredictor::getDelay, ""));
}

sot::VectorMultiBound &TaskSmithPredictor::computeTaskExponentialDecrease(
    sot::VectorMultiBound &errorRef, int time) {
  const Vector &error = errorSOUT(time);
  const double &gain = controlGainSIN(time);
  errorRef.resize(error.size());

  if (!initialized) {
    for (std::size_t i = 0; i < inputMem.size(); ++i) {
      inputMem[i] = Vector::Zero(error.size());
      outputMem[i] = Vector::Zero(error.size());
    }
    initialized = true;
  }

  // Vector output(gain * ((-error) - inputMem.back()) -
  //               (gain * period - 1.) * outputMem.front() +
  //               gain * period * outputMem.back());
  Vector output ((gain * ((-error) - inputMem.front())
                + outputMem.front() + gain * period * outputMem.back()) / (1 + gain * period) );

  for (unsigned int i = 0; i < errorRef.size(); ++i) errorRef[i] = output[i];

  // Update memory
  inputMem.push_front(-error);
  outputMem.push_front(output);

  /* TODO does this make sense ?
   * It accounts for possible motions of the reference so it should be taken
   * into accounts but how ?
  if (withDerivative) {
    const dynamicgraph::Vector &de = errorTimeDerivativeSOUT(time);
    for (unsigned int i = 0; i < errorRef.size(); ++i)
      errorRef[i] = errorRef[i].getSingleBound() - de(i);
  } */

  return errorRef;
}

void TaskSmithPredictor::initialize(const double& T, const int& delay) {
  period = T;

  inputMem.resize(delay);
  outputMem.resize(delay);

  initialized = false;
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(TaskSmithPredictor, "TaskSmithPredictor");
}  // namespace agimus
}  // namespace dynamicgraph
