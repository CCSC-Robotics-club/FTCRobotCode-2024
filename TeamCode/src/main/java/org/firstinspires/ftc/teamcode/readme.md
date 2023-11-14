# Chassis code for 2024
### team 19581 "IRON MAPLE" presents

## TODO
- add a param to the april tag cam, the rotation of the pointing of the camera
- add on-release function to pilot controller
- write a framework for auto programs
## schedule
- this afternoon, finish tuning the visual approach, make it to the best

## aiming
- perhaps we should change the way of aiming
- we can sense the position of the wall for once and go to a position in front of it by encoder, and face front in the mean time
- then we adjust the robot for horizontal position, and approach to the correct distance with TOF sensor, face 0 position during this entire process
- test had made it clear that the IMU is very accurate

## passing through the obstacle
- we can set a particular position as the port to pass the field
- when the robot pass through the ports, we record the entrance and exit position and measure the x coordinate of the port
- when the pilot presses a button, the robot lines up with the port automatically