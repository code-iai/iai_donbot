# troubeshooting
## power switches:
- 1 ur5
- 2 & 3 ethercat wheels
- 4 homer

# How to start Donbot
- turn on switch 1 for ur5 and start it on the touch pad
- take ps controller
- unplug donbot from the power supply
- take e-stop and unlock
- ``roscore``
- ``roslaunch iai_donbot_bringup donbo.launch``
- start rviz and localize robot
- on touchpad start remove control app (you may have to restart donbots launch file afterwards)
- make sure you don't run out of battery! it damages them! They last about 6-8hours, depending on what you do. The launch file starts a watchdog and will print warnings, if you have to recharge.

## Troubleshooting
- cameras don't work
  - unplug them and plug them back in
- robot doesn't driver
  - sometimes the e-stop uses connection and stops the robot (you would hear a click), just unlock again  
  - check terminal, if it says: ``sudo /etc/init.d/ethercat restart`` kill the launch file, execute this command, wait a bit and try again
  - check if the controller is connected and messages appear on the /joy topic
  - if nothing else works, restart the wheels with power switches 2 and 3
- the arm doesn't move
  - check the touchpad, sometimes it goes into a soft stop, when the velocities are too high
