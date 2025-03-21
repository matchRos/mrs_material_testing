#!/bin/bash
roslaunch mrs_material_testing multi_mur620_handling.launch &
sleep 60  # Warte 5 Sekunden
roslaunch mrs_material_testing multi_decentralized_admittance_controller_sim.launch &
rosrun mrs_material_testing switch_URs_to_twist_control.py & 
roslaunch mrs_material_testing multi_twist_controller_sim.launch 

