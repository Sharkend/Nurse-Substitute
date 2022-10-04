search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=arm.srdf
robot_name_in_srdf=arm
moveit_config_pkg=arm_moveit_config
robot_name=arm
planning_group_name=arm
ikfast_plugin_pkg=arm_arm_ikfast_plugin
base_link_name=base_link
eef_link_name=flange_adaptor_2
ikfast_output_path=/home/sharpop/Documents/crais_ws/src/arm_arm_ikfast_plugin/src/arm_arm_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
