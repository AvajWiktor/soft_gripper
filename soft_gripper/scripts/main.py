import sys

from view import main_view

if __name__ == '__main__':

    App = main_view.MainWindowView()
    App.root.destroy()
    #rospy.spin()
    sys.exit("EXIT SUCCESS")
