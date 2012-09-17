#!/usr/bin/env python
import sys

import roslib; roslib.load_manifest('knex_ros')
import rospy
from geometry_msgs.msg import Twist


from PySide import QtGui, QtCore

##########################################################################
##########################################################################
class MainWindow(QtGui.QMainWindow):
##########################################################################
##########################################################################

    #####################################################################    
    def __init__(self):
    #####################################################################    
        super(MainWindow, self).__init__()
        self.timer_rate = rospy.get_param('~publish_rate', 25)
        self.pub_twist = rospy.Publisher('twist', Twist)
        
        self.initUI()
        
    #####################################################################    
    def initUI(self):      
    #####################################################################    

        self.statusBar()
        
        self.setStyleSheet("QMainWindow { border-image: url(crosshair.jpg); }")
        
                
        self.setGeometry(0, 600, 200, 200)
        self.setWindowTitle('Virtual Joystick')
        self.show()
        self.timer = QtCore.QBasicTimer()
        
        self.statusBar().showMessage('started')
        
    #####################################################################    
    def mousePressEvent(self, event):
    #####################################################################    
        self.statusBar().showMessage('mouse clicked')
        self.timer.start(self.timer_rate, self)
        
    #####################################################################    
    def mouseReleaseEvent(self, event):
    #####################################################################    
        self.statusBar().showMessage('mouse released')
        self.timer.stop()
        
    #####################################################################    
    def mouseMoveEvent(self, event):
    #####################################################################    
        s = self.size()
        s_w = s.width()
        s_h = s.height()
        pos = event.pos()
        self.x = 1.0 * pos.x() / s_w
        self.y = 1.0 * pos.y() / s_h
        
        self.statusBar().showMessage('point (%0.2f, %0.2f)' % (self.x,self.y))
        
    #####################################################################    
    def timerEvent(self, event):
    #####################################################################    
        # self.statusBar().showMessage("timer tick")
        self.pubTwist()
        
    #######################################################
    def pubTwist(self):
    #######################################################
        rospy.loginfo("publishing twist from (%0.3f,%0.3f)" %(self.x,self.y))
        self.twist = Twist()
        self.twist.linear.x = self.y
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = self.x
        self.pub_twist.publish( self.twist )
        
##########################################################################
##########################################################################
def main():
##########################################################################
##########################################################################
    rospy.init_node('virtual_joystick')
    rospy.loginfo('virtual_joystick started')
    
    app = QtGui.QApplication(sys.argv)
    ex = MainWindow()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
