#include <boost/circular_buffer.hpp>

#include <sot/core/task.hh>

namespace dynamicgraph {
namespace agimus {

class TaskSmithPredictor : public sot::Task {
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  TaskSmithPredictor(const std::string &n);

  sot::VectorMultiBound &computeTaskExponentialDecrease(
      sot::VectorMultiBound &errorRef, int time);

  void initialize(const double& period, const int& delay);

  double getPeriod() { return period; }
  unsigned getDelay() { return (unsigned) inputMem.size(); }

  /* The above funcs should be as below but are kept as above for compat with commands.
  double getPeriod() const { return period; }
  std::size_t getDelay() const { return inputMem.size(); }
  */

 private:
  boost::circular_buffer<Vector> inputMem, outputMem;
  double period;

  bool initialized;
};  // class TaskSmithPredictor

}  // namespace agimus
}  // namespace dynamicgraph
