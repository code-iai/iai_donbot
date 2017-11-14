# iai_donbot
A collection of description, launch, configuration, and installation files for the Donbot robot at the Institute for Artificial Intelligence. 

## Installation instructions for 16.04/kinetic

```
source /opt/ros/kinetic/setup.bash          # start using ROS kinetic
mkdir -p ~/homer_ws/src                     # create directory for workspace
cd ~/homer_ws                               # go to workspace directory
catkin init                                 # init workspace
cd src                                      # go to source directory of workspace
wstool init                                 # init rosinstall
wstool merge https://raw.githubusercontent.com/code-iai/iai_donbot/master/rosinstall/catkin.rosinstall
                                            # update rosinstall file
wstool update                               # pull source repositories
rosdep install --ignore-src --from-paths .  # install dependencies available through apt
cd ..                                       # go to workspace directory
catkin build                                # build packages
catkin build                                # build again because img_eml will fail the first time :(
source ~/homer_ws/devel/setup.bash          # source new overlay
```

## Start DonBot

This has to be executed on homer:
```
roslaunch iai_donbot_bringup bringup.launch
```
If the ur5 is not turned on, you will see some error messages but the base will work just fine.