#!/usr/bin/env python
import sys
import rospy
import math
import tf
import geometry_msgs.msg

p_angular = 0.25
min_angular_speed = 0.02
max_angular_speed = 0.1
angular_cutoff = 0.01

p_linear = 0.25
min_linear_speed = 0.01
max_linear_speed = 0.1
linear_cutoff = 0.05

target_dist = 0.8

def approach_main(tag_name):
  global listener
  global cmd_pub
  rospy.init_node('approach_tag')
  listener = tf.TransformListener()
  cmd_pub = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
  
  align_tag(tag_name)
  approach_tag(tag_name)

def absclamp(val, minval, maxval):
  newval = val
  if abs(newval) > maxval:
    newval = math.copysign(maxval, newval)
  if abs(newval) < minval:
    newval = math.copysign(minval, newval)
  return newval

def align_tag(tag_name):
  listener.waitForTransform('camera_color_frame', tag_name, rospy.Time(0), rospy.Duration(5.0))
  while not rospy.is_shutdown():
    (fw_trans,fw_rot) = listener.lookupTransform('camera_color_frame', tag_name, rospy.Time(0))
    (rev_trans,rev_rot) = listener.lookupTransform(tag_name, 'camera_color_frame', rospy.Time(0))

    fw_angle = math.atan2(fw_trans[0],fw_trans[2])
    angular = absclamp(-p_angular * fw_angle, min_angular_speed, max_angular_speed)

    rev_angle = math.atan2(rev_trans[0], rev_trans[2])
    liny = absclamp(-p_angular * rev_angle, min_linear_speed, max_linear_speed)
    print('FAngle: {:f} ang. speed: {:f} RAngle: {:f} lin. speed: {:f}'.format(fw_angle, angular, rev_angle, liny))

    cmd = geometry_msgs.msg.Twist()
    if abs(fw_angle) <= angular_cutoff and abs(rev_angle) <= angular_cutoff:
      cmd_pub.publish(cmd)
      break;

    cmd.linear.y = liny
    cmd.angular.z = angular
    cmd_pub.publish(cmd)

def approach_tag(tag_name):
  listener.waitForTransform('camera_color_frame', tag_name, rospy.Time(0), rospy.Duration(5.0))
  while not rospy.is_shutdown():
    (trans,rot) = listener.lookupTransform('camera_color_frame', tag_name, rospy.Time(0))
    distance = trans[2] - target_dist
    linx = absclamp(-p_linear * distance, min_linear_speed, max_linear_speed)
    print('Dist: {:f} lin. speed: {:f}'.format(distance, linx))

    cmd = geometry_msgs.msg.Twist()
    if distance <= linear_cutoff:
      cmd_pub.publish(cmd)
      break

    cmd.linear.x = linx
    cmd_pub.publish(cmd)
    

if __name__ == '__main__':
  if len(sys.argv) < 2:
    print("Missing tag name in command line arguments!")
    sys.exit(0)

  try:
    approach_main(sys.argv[1])
  except rospy.ROSInterruptException:
    pass
