#!/usr/bin/env python
import roslib; roslib.load_manifest('kinex_ros')
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16
import wx


######################################################################
######################################################################
class MainWindow(wx.Frame):
    """ the main window """
    
    ######################################################################
    def set_status_text(self, txt):
    ######################################################################
        wx.CallAfter(self.tbArduinoDebug.SetValue, txt)

    ######################################################################
    def arduino_debug_callback(self, data):
    ######################################################################
        self.set_status_text(data.data)
        
    ######################################################################
    def __init__(self, parent, title):
    ######################################################################
        wx.Frame.__init__(self, parent, title=title, size=(350,200))
        panel = wx.Panel(self, -1)
        self.top_sizer = wx.BoxSizer(wx.VERTICAL)
        
        self.tbArduinoDebug = wx.TextCtrl(panel)
        self.lblDebug = wx.StaticText(panel, -1, "debug", (150,50))

        self.row1_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.row1_sizer.Add(self.lblDebug)
        self.row1_sizer.Add(self.tbArduinoDebug)
        
        
        
        self.lblSliderL = wx.StaticText(panel, -1, "Left", (25,50), style=wx.ALIGN_RIGHT)
        self.sldLeft = wx.Slider(panel, -1, 0, -255, 255, wx.DefaultPosition, (250, 50),
                             wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS)
        
        self.row2_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.row2_sizer.Add(self.lblSliderL)
        self.row2_sizer.Add(self.sldLeft)
        
        self.lblSliderR = wx.StaticText(panel, -1, "Right", (25,50), style=wx.ALIGN_RIGHT | wx.ALIGN_CENTER_VERTICAL)
        self.sldRight = wx.Slider(panel, -1, 0, -255, 255, wx.DefaultPosition, (250,50),
                                  wx.SL_AUTOTICKS | wx.SL_HORIZONTAL | wx.SL_LABELS)
        self.sldLeft.Bind(wx.EVT_SLIDER,self.OnSliderLeft)
        self.sldRight.Bind(wx.EVT_SLIDER,self.OnSliderRight)
        
        self.row3_sizer=wx.BoxSizer(wx.HORIZONTAL)
        self.row3_sizer.Add(self.lblSliderR)
        self.row3_sizer.Add(self.sldRight)
        
        self.btnZero = wx.Button(panel, -1, "Zero", (25,50))
        self.btnZero.Bind(wx.EVT_BUTTON, self.OnButtonZero)
        
        self.row4_sizer = wx.BoxSizer( wx.HORIZONTAL )
        self.row4_sizer.Add( self.btnZero )
        
        self.top_sizer.Add(self.row1_sizer)
        self.top_sizer.Add(self.row2_sizer)
        self.top_sizer.Add(self.row3_sizer)
        self.top_sizer.Add(self.row4_sizer)
        
        panel.SetSizer(self.top_sizer)
                
        pub = rospy.Publisher('chatter', String)
        self.pub_lmotor = rospy.Publisher('lmotor_cmd', Int16)
        self.pub_rmotor = rospy.Publisher('rmotor_cmd', Int16)
        rospy.init_node('kinex_arduino_connector')
        rospy.Subscriber("arduino_debug", String, self.arduino_debug_callback)    
        
        self.joy = wx.Joystick(1)
        self.joy.SetCapture(self)
        self.Bind(wx.EVT_JOY_BUTTON_DOWN, self.OnJoyBtn)
        self.Bind(wx.EVT_JOY_BUTTON_UP, self.OnJoyBtn)
        self.Bind(wx.EVT_JOY_MOVE, self.OnJoyMove)
        rospy.loginfo ( "done initializing") 
        
    ######################################################################
    def OnJoyBtn(self, event):
    ######################################################################
        rospy.loginfo( "Joystick button pressed" )
        
    ######################################################################
    def OnJoyMove(self, event):
    ######################################################################
        pos = self.joy.GetPosition()
        max = self.joy.GetXMax()
        rospy.loginfo( "Joystick move x=%d, y=%d max=%d" % (pos[0], pos[1], max) )
        self.diff_drive(float(pos[0]) / max, float(pos[1]) / max)
        
        
    ######################################################################
    def diff_drive(self, x, y):
    ######################################################################
        rospy.loginfo("diffdrive x=%0.3f, y=%0.3f" % (x,y))
        threshold = 0.1
        if ( abs(x) < threshold and abs(y) < threshold ):
            l = 0
            r = 0
            
        else:
            if( x >= 0):
                rmin = x - 1  # -1 to 0
                rmax = 1
                lmin = -1
                lmax = 1 - x  # 1 to 0
            else:
                rmin = -1
                rmax = 1 + x  # 1 to 0
                lmin = -1 - x # -1 to 0
                lmax = 1
        
            l = lmin + ( lmax - lmin ) / 2 + y * (lmax - lmin) / 2 * -1
            r = rmin + ( rmax - rmin ) / 2 + y * (rmax - rmin) / 2 * -1
            rospy.loginfo( " l:%0.2f(%0.2f:%0.2f) r:%0.2f(%0.2f:%0.2f)" % (l,lmin,lmax,r,rmin,rmax))
        
        wx.CallAfter( self.sldLeft.SetValue, l * 255)
        wx.CallAfter( self.sldRight.SetValue, r * 255)
        self.pub_lmotor.publish( l * 255 )
        self.pub_rmotor.publish( r * 255)
        
    ######################################################################
    def OnSliderLeft(self, evt):
    ######################################################################
        rospy.loginfo ( "Left slider value %d" % self.sldLeft.GetValue() )
        wx.CallAfter(self.pub_lmotor.publish, self.sldLeft.GetValue())
        
    ######################################################################
    def OnSliderRight(self, evt):
    ######################################################################
        rospy.loginfo ( "Right slider value %d" % self.sldRight.GetValue() )
        wx.CallAfter(self.pub_rmotor.publish, self.sldRight.GetValue())
        
    ######################################################################
    def OnButtonZero(self, evt):
    ######################################################################
        rospy.loginfo("Button Zero")
        wx.CallAfter( self.sldLeft.SetValue, 0)
        wx.CallAfter( self.sldRight.SetValue, 0)
        self.pub_rmotor.publish(0)
        self.pub_lmotor.publish(0)
        
        


######################################################################
def launch_window():
######################################################################
    """ launches the GUI and nothing else """
        
    app = wx.App(False)  # Create a new app, don't redirect stdout/stderr to a window.
    frame = MainWindow(None, "Kinex arduino connector") # A Frame is a top-level window.
    frame.Show(True)     # Show the frame.
    app.MainLoop()

    while not rospy.is_shutdown():
        str = "hello world %s"%rospy.get_time()
        rospy.loginfo(str)
        pub.publish(String(str))
        rospy.sleep(1.0)
        
        
        
######################################################################
######################################################################
if __name__ == '__main__':
    try:
        launch_window()
    except rospy.ROSInterruptException: pass