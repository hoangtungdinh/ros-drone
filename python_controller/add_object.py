import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.msg import PlanningScene

## END_SUB_TUTORIAL

from std_msgs.msg import String

def move_group_python_interface_tutorial():
  ## BEGIN_TUTORIAL
  ##
  ## Setup
  ## ^^^^^
  ## CALL_SUB_TUTORIAL imports
  ##
  ## First initialize moveit_commander and rospy.
  print "============ Starting tutorial setup"
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)


  rospy.sleep(2)
  ## Instantiate a RobotCommander object.  This object is an interface to
  ## the robot as a whole.
  robot = moveit_commander.RobotCommander()

  ## Instantiate a PlanningSceneInterface object.  This object is an interface
  ## to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  p = geometry_msgs.msg.PoseStamped()
  p.header.frame_id = robot.get_planning_frame()
  print robot.get_planning_frame()
  p.pose.position.x = 0.0
  p.pose.position.y = 10.0
  p.pose.position.z = 0.0

  rate = rospy.Rate(10)

  for i in range(10):
    # scene.add_box('aBox', p, (2,2,2))
    scene.add_mesh('simple_pole', p, 'simple_pole.stl')
    rate.sleep()


  # scene_pub.publish(PlanningScene)
  print "DONE==================="


if __name__=='__main__':
  try:
    move_group_python_interface_tutorial()
  except rospy.ROSInterruptException:
    pass
