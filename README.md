# baxter_pick_ball

using opencv hough circle to find the ball, move the arm horizonticall close to it.
repeat this process untill the position of the ball in the middle of the image.
execte picking

* gripper control 
* image processing
  * receive image
  * process image
  * recognize a ball 
  * recognize fruit
* arm control
  * inverse kinematic transformation
  * offset values
    * camera-Baxter offset
    * arm offset
