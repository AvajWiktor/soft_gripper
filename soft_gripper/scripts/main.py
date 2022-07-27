import sys

import rospy
from view import main_view

if __name__ == '__main__':

    rospy.init_node("gripper", anonymous=True)
    App = main_view.MainWindowView()
    App.root.destroy()
    #rospy.spin()
    sys.exit("EXIT SUCCESS")