| Prev:  [Hacking the motors](HackingTheMotors.md)| [-Top-](ProjectOverview.md) | Next:  [First Arduino code](FirstArduinoCode.md) |
|:------------------------------------------------|:----------------------------|:-------------------------------------------------|
# Connecting the motors to an Arduino #

The next step is to get the Arduino to control the motors.

I built my own dual H-bridge circuit because I thought it would be much cheaper than buying one.  I based mine off of [this tutorial](http://www.mcmanis.com/chuck/robotics/tutorial/h-bridge/bjt-circuit.html) mainly because it had opto-isolated inputs.  I would not recommend this - it took me a very long time, and I ended up with quite a messy circuit.

Seeedstudio makes a motor shield (Radio Shack [carries it](http://www.radioshack.com/product/index.jsp?productId=13351656)), which would probably work just as well.


# Details #

I used male headers (from [RobotShop](http://www.robotshop.com/productinfo.aspx?pc=RB-Spa-153&lang=en-US) to connect to the K'nex connectors.  Some more hot glue helped for some structural support.

<img src='https://lh4.googleusercontent.com/-FI1tUpzu1O0/UEF5ukjur1I/AAAAAAAABWA/c-v98NdMkNM/s860/2012-08-31_22-58-09_401.jpg' height='175' width='300'>

I used some Cat 5 cable I happened to have laying around, which happens to have 4 twisted pairs.  Here's my mess of a dual H-Bridge shield:<br>
<br>
<img src='https://lh5.googleusercontent.com/-s43y_FGpryw/UEF6AXmMIpI/AAAAAAAABWQ/gpauw_fbgYc/s860/2012-08-31_22-59-20_706.jpg' height='175' width='300'>

At this point the schematic will look like this:<br>
<img src='https://lh4.googleusercontent.com/-K_fvE_09AHA/UGe1FFdX2LI/AAAAAAAABfg/7VL4aF2AcnU/s605/robot_schematic_2.png'>

Note that I'm using some of the analog input pins as digital outputs.  That's because of some constraints I ran into later on in the project.  For now let's just say it had to do with servos, pulse width modulation, and counters.  I'll come back to that.<br>
<br>
Next:  <a href='FirstArduinoCode.md'>First Arduino code</a>