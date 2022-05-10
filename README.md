# baxter_pick_ball

using opencv hough circle to find the ball

repeatedly measure the distance between the ball and target point in the image, and offset the limb's position

execute picking

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
