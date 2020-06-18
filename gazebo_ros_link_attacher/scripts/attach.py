#!/usr/bin/env python

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse


if __name__ == '__main__':
    rospy.init_node('demo_attach_links')
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                    Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

    # Link them
    rospy.loginfo("Attaching cube1 and cube2")
    req = AttachRequest()
    req.model_name_1 = "abb_irb6640_185_280"
    req.link_name_1 = "link_6"
    req.model_name_2 = "box_red"
    req.link_name_2 = "link"
    

    attach_srv.call(req)
    # From the shell:
    """
rosservice call /link_attacher_node/attach "model_name_1: 'abb_irb6640_185_280'
link_name_1: 'link_6'
model_name_2: 'box_red'
link_name_2: 'link'"
    """
    #rospy.loginfo("Attaching cube2 and cube3")
    #req = AttachRequest() 
    #req.model_name_1 = "box_blue"
    #req.link_name_1 = "link"
    #req.model_name_2 = "box_red"
    #req.link_name_2 = "link"

   # attach_srv.call(req)
    # From the shell:
 #   """
#rosservice call /link_attacher_node/attach "model_name_1: 'box_blue'
#link_name_1: 'link'
#model_name_2: 'box_red'
#link_name_2: 'link'"
#    """
  
