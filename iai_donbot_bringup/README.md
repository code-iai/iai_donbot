# troubeshooting
## power switches:
- 1 ur5
- 2 & 3 ethercat wheels
- 4 homer

# How to start marcos fingers

## setup fingers
- first, connect fingers with power cable: see jpg picture in /launch
  (this plugs them into the supply. It needs +24V.
- next, connect the usb cables of the fingers into the black usb hub
  IMPORTANT:  You need to connect the USB of the fingers in a specific order!
  F001 first and then F002. This way, the OS will associate /dev/ttyUSB0 to F001 and ttyUSB1 to F002.

## setup gripper
- on wsg50 website (http://192.168.102.63/) stop running script:
  Settings -> System -> Startup script -> terminate -> Abort
- on wsg50 website: scripting -> interactive scripting -> open -> cmd_vel_out.lua -> run
- on wsg50 website: click Ack on the right column

## launch fingers and gripper
- source /home/refills/ws_fingers/devel/setup.bash
- roslaunch slipping_control_wsg50_launch slipping_control_tulip.launch

## launch giskard fingers integration script
- Make sure the arm is running, the new driver stuff is on "Play"
- source /home/refills/ws_fingers/devel/setup.bash
- roslaunch giskard_donbot_finger_hack refills_finger_hack.launch


## troubleshooting
- "Failed to read data from TCP socket": restart gripper

- Before starting slipping_control..._launch, make sure you pressed Ack on the wsg50 wegsite
