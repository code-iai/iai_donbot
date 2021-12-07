cmake_minimum_required(VERSION 3.0.2)
project(iai_donbot_sim)

find_package(catkin REQUIRED COMPONENTS
  iai_donbot_description
  iai_naive_kinematics_sim
  robot_state_publisher
  omni_pose_follower
  tf2_ros
)

catkin_package()

include_directories(
# include
#   ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
  scripts/refills_finger_fake_traj_server.py
  scripts/refills_finger_js_publisher.py
  scripts/navp_faker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
