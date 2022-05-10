# baxter_pick_ball
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/Pw_PhkqKUx4/0.jpg)](https://www.youtube.com/watch?v=Pw_PhkqKUx4)

using opencv hough circle to find the ball

repeatedly measure the distance between the ball and target point in the image, and offset the limb's position to decrease the distance

execute ball picking

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
