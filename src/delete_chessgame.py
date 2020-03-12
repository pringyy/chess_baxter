#!/usr/bin/env python
import sys, rospy, tf, rospkg
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from copy import deepcopy

# http://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials

if __name__ == '__main__':
    rospy.init_node("delete_chessboard")
    rospy.wait_for_service("gazebo/delete_model")

    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)

    list_pieces = rospy.get_param('list_pieces')
    board_setup = rospy.get_param('board_setup')
    
    for row, each in enumerate(board_setup):
        for col, piece in enumerate(each):
            if piece in list_pieces:
                piece_name = "%s%d" % (piece, col)
                print "Deleting "+piece_name
                delete_model(piece_name)

    delete_model("cafe_table")
    delete_model("chessboard")
