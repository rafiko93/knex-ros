# Introduction #

My experiments with my 3d camera and ROS


# Details #

I am using the [Minoru 3d webcam](http://www.robotshop.com/productinfo.aspx?pc=RB-Pul-01&lang=en-US).  It was cheap.  It works in linux out of the box, when plugged in, two new devices show up as /dev/video1 and /dev/video2.

SLAM systems:
  * RGBDSLAM from rgbdslam\_freiburg.
    * Cannot get it to compile.
  * VSLAM from vslam\_system
    * Works, but the SLAM is terrible, and it crashes after a few dozen frames, possibly when the image is blurred.
  * PTAM
    * apparently works on mono data
