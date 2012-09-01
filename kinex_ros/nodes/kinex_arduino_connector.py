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
        
        self.top_sizer.Add(self.row1_sizer)
        self.top_sizer.Add(self.row2_sizer)
        self.top_sizer.Add(self.row3_sizer)
        
        panel.SetSizer(self.top_sizer)
                
        pub = rospy.Publisher('chatter', String)
        self.pub_lmotor = rospy.Publisher('lmotor_cmd', Int16)
        self.pub_rmotor = rospy.Publisher('rmotor_cmd', Int16)
        rospy.init_node('kinex_arduino_connector')
        rospy.Subscriber("arduino_debug", String, self.arduino_debug_callback)    
        rospy.loginfo ( "done initializing") 
        
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