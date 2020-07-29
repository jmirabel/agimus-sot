solver = supervisor.preActions['driller/drill_tip > skin/hole | 0-0_12']
task = solver.tasks[0]
from dynamic_graph.ros.ros_publish import RosPublish
publish = RosPublish("ros_publish_values")
publish.add('vector', 'error', '/sot/task_error')
from dynamic_graph import plug
plug(task.error, publish.error)
robot.device.after.addSignal("ros_publish_values.trigger")

