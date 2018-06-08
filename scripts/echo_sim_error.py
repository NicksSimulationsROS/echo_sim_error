#!/usr/bin/env python
import rospy
import math
from std_msgs.msg       import Float32
from nav_msgs.msg       import Odometry
from gazebo_msgs.msg    import ModelStates
from tf.transformations import euler_from_quaternion

class echo_sim_error:
  model_states_msg = None

  def __init__(self):
    self.last_echo_time = rospy.get_rostime()
    self.params()
    self.pubs()
    self.subs()

  def params(self):
    self.model_name = rospy.get_param('~model_name', 'none')
    self.echo_rate  = rospy.get_param('~echo_rate', 1)
    print("model name: " + self.model_name)

  def subs(self):
    rospy.Subscriber("model_states", ModelStates, self.modelStatesCallback)
    rospy.Subscriber(    "odometry",    Odometry, self.odometryCallback)

  def pubs(self):
    self.pub = rospy.Publisher('error', Float32, queue_size=10)

  def modelStatesCallback(self, model_states_msg):
    self.model_states_msg = model_states_msg

  def odometryCallback(self, odometry_msg):
    if rospy.get_rostime() - self.last_echo_time < rospy.Duration(1/self.echo_rate):
      return
    if self.model_states_msg == None:
      return

    # Find our model.
    found_match = False
    names = self.model_states_msg.name
    poses = self.model_states_msg.pose
    for name,pose in zip(names,poses):
      if name == self.model_name:
        found_match = True
        break
    if not found_match:
      print('Unable to find ' + self.model_name + ' in model states')
      return
    # Calculate error.
    x      = odometry_msg.pose.pose.position.x
    y      = odometry_msg.pose.pose.position.y
    quat   = (odometry_msg.pose.pose.orientation.x,
              odometry_msg.pose.pose.orientation.y,
              odometry_msg.pose.pose.orientation.z,
              odometry_msg.pose.pose.orientation.w)
    euler = euler_from_quaternion(quat)
    yaw   = euler[2]*180/3.1415
    x_true = pose.position.x
    y_true = pose.position.y
    quat_true = (pose.orientation.x,
                 pose.orientation.y,
                 pose.orientation.z,
                 pose.orientation.w)
    euler_true = euler_from_quaternion(quat_true)
    yaw_true   = euler_true[2]*180/3.1415
    dist_err = math.sqrt( (x_true-x)**2 + (y_true-y)**2 )
    yaw_err  = yaw_true - yaw;
    while yaw_err < -180:
      yaw_err += 360
    while yaw_err > 180:
      yaw_err -= 360
    #print('Estimated pose: ' + '%0.1f'%x +      ' , ' + '%0.1f'%y      + ' , ' + '%0.1f'%yaw)
    #print('True pose:      ' + '%0.1f'%x_true + ' , ' + '%0.1f'%y_true + ' , ' + '%0.1f'%yaw_true)
    #print('Distance error: ' + '%0.1f'%dist_err)

    nice_name = '{:<15}'.format(self.model_name[:15])
    nice_dist = '{:<15}'.format('%0.2f'%dist_err)
    nice_yaw  = '{:<15}'.format('%0.2f'%yaw_err)
    print('------------------------------------------------------')
    print('| model name      | dist error (m)  | yaw error (deg) |')
    print('| '+ nice_name +' | '+ nice_dist +' | '+ nice_yaw + ' |')#, end='\r')
    self.last_echo_time = rospy.get_rostime()
    # Publish.
    self.pub.publish(dist_err)

if __name__ == '__main__':
  print("Initialising node")
  rospy.init_node('echo_sim_error', anonymous=True)
  print("Creating object")
  obj = echo_sim_error()
  rospy.spin()
