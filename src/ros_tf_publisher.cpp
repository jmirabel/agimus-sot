#include <pinocchio/fwd.hpp>
#include <boost/bind.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/command-bind.h>
#include <dynamic-graph/factory.h>

#include <dynamic_graph_bridge/ros_init.hh>

#include <sot/core/matrix-geometry.hh>

namespace dynamicgraph {
using sot::MatrixHomogeneous;

class RosTfPublisher : public Entity {
  DYNAMIC_GRAPH_ENTITY_DECL();

 public:
  RosTfPublisher(const std::string& name)
    : Entity(name)
    , trigger("RosTfPublisher("+name+")::input(int)::trigger")
    , tfBr()
  {
    trigger.setFunction (boost::bind(&RosTfPublisher::triggerPublish, this, _1, _2));
    signalRegistration(trigger);

    std::string docstring =
        "\n"
        "  Add a signal containing the transform between two frames.\n"
        "\n"
        "  Input:\n"
        "    - to  : frame name\n"
        "    - from: frame name,\n"
        "    - signalName: the signal name in dynamic-graph"
        "\n";
    addCommand("add", command::makeCommandVoid3(*this, &RosTfPublisher::add, docstring));
  }

  ~RosTfPublisher() {}

  void add(const std::string& to, const std::string& from, const std::string& signame);

  inline void publish (int sotTime)
  {
    publish (sotTime, ros::Time::now());
  }

  void publish (int sotTime, const ros::Time& rosTime);

 private:
  int& triggerPublish (int& value, int time)
  {
    publish (time);
    value = 0;
    return value;
  }

  struct Data {
    geometry_msgs::TransformStamped msg;
    SignalPtr<MatrixHomogeneous, int> signal;

    Data (const std::string& to, const std::string& from, const std::string& signame);
  };

  Signal<int, int> trigger;

  typedef std::map<std::string, boost::shared_ptr<Data> > Map_t;
  Map_t datas;
  tf2_ros::TransformBroadcaster tfBr;
};

void RosTfPublisher::publish(int sotTime, const ros::Time& rosTime)
{
  for (Map_t::iterator it = datas.begin(); it != datas.end(); ++it) {
    geometry_msgs::TransformStamped& msg (it->second->msg);
    SignalPtr<MatrixHomogeneous, int>& signal (it->second->signal);
    signal.recompute(sotTime);
    const MatrixHomogeneous& M (signal.accessCopy());

    msg.header.stamp = rosTime;

    msg.transform.translation.x = M.translation()[0];
    msg.transform.translation.y = M.translation()[1];
    msg.transform.translation.z = M.translation()[2];

    Eigen::Quaterniond q (M.linear());
    msg.transform.rotation.x = q.x();
    msg.transform.rotation.y = q.y();
    msg.transform.rotation.z = q.z();
    msg.transform.rotation.w = q.w();

    tfBr.sendTransform(msg);
  }
}

void RosTfPublisher::add(const std::string& to, const std::string& from, const std::string& signame)
{
  if (datas.find(signame) != datas.end())
    throw std::invalid_argument("A signal " + signame + " already exists in RosTfPublisher " + getName());

  boost::format signalName("RosTfPublisher(%1%)::input(MatrixHomo)::%2%");
  signalName % getName() % signame;

  boost::shared_ptr<Data> tld (boost::make_shared<Data>(to, from, signame));
  signalRegistration(tld->signal);
  datas[signame] = tld;
}

RosTfPublisher::Data::Data(const std::string& to, const std::string& from, const std::string& signame)
: signal (NULL, signame)
{
  msg.header.frame_id = from;
  msg.child_frame_id = to;
}

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(RosTfPublisher, "RosTfPublisher");
}  // namespace dynamicgraph

