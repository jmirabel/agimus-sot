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

  void initialize(double period, int delay);

  double getPeriod() const { return period; }
  std::size_t getDelay() const { return inputMem.size(); }

 private:
  boost::circular_buffer<Vector> inputMem, outputMem;
  double period;

  bool initialized;
};  // class TaskSmithPredictor

}  // namespace agimus
}  // namespace dynamicgraph
