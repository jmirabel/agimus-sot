solver = supervisor.preActions['driller/drill_tip > skin/hole | 0-0_12']
task = solver.tasks[0]
from dynamic_graph.ros.ros_publish import RosPublish
publish = RosPublish("ros_publish_values")
publish.add('vector', 'error', '/sot/task_error')
from dynamic_graph import plug
plug(task.error, publish.error)
robot.device.after.addSignal("ros_publish_values.trigger")


import dynamic_graph as dg
feature = dg.entity.Entity.entities["pregrasp___skin/hole___driller/drill_tip___based___gripper___driller/handle_feature"]
import agimus_sot
rosTfPub = agimus_sot.sot.RosTfPublisher("ros_tf_publisher")
agimus_sot.tools.visualizeFeaturePose(feature, rosTfPub, "/sot/featurePose")
robot.device.after.addSignal("ros_tf_publisher.trigger")

from dynamic_graph.ros.ros_publish import RosPublish
from dynamic_graph import plug
solver = supervisor.preActions['driller/drill_tip > skin/hole | 0-0_12']
task = solver.tasks[0]
ros_publish = RosPublish("ros_publish")
ros_publish.add("vector", "control", "/sot/control")
ros_publish.add("vector", "vs_error", "/sot/vs_error")
plug(robot.device.control, ros_publish.control)
plug(task.error, ros_publish.vs_error)
robot.device.after.addSignal("ros_publish.trigger")
solver.setLimits(robot)

