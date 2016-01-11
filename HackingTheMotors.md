| [-Top-](ProjectOverview.md) | Next:  [Hooking the motors up to an Arduino](MotorsToArduino.md) |
|:----------------------------|:-----------------------------------------------------------------|

# Hacking the motors #

The very first step I took was hacking a pair of K'nex motors to make them controllable by an Arduino.

# Details #

There are already excellent instructions here: http://www.instructables.com/id/Arduino-Robot-for-lowest-cost/.  I essentially did the same thing as steps 1-3 outlined there.

The only difference is that I used the existing K'nex batteries to power my motors.  The schematic will end up looking like this:
<img src='https://lh3.googleusercontent.com/-EgWtJsP35x4/UGZlAgGltoI/AAAAAAAABdI/B5De9YKCZMU/s696/knex_motors.png' height='175' width='500'>

This way I get 6V out of the batteries, even though the original K'nex motors only ran on 3V.  This should help compensate for the voltage drop across the H-bridge circuit, but it may shorten the motor life.  Here's a picture of the battery connection:<br>
<br>
<img src='https://lh3.googleusercontent.com/-CHmlqjkm94c/UEF4ukq-uYI/AAAAAAAABVo/8Rrq0wAnGGs/s805/2012-08-31_22-53-48_242.jpg' height='175' width='300'>

I used some female headers (from <a href='http://www.robotshop.com/productinfo.aspx?pc=RB-Spa-346&lang=en-US'>RobotShop</a>) to make a connection on the top of the motor assembly to make it slightly easier to work with.  These are glued on with a high-temp glue gun.<br>
<br>
<img src='https://lh4.googleusercontent.com/-lAIsSx5oiJI/UEF5W3r4BnI/AAAAAAAABV4/OMYkJdObFmc/s805/2012-08-31_22-56-16_256.jpg' height='175' width='300'>

Next: <a href='MotorsToArduino.md'>Hooking the motors up to an Arduino</a>