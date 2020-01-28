# Example: Visualize Kinematics
<p align="center">
  <img src="graphics/visualize_kinematics.gif" height="300" >
</p>

This example demonstrates the ability of the joint-mechanics tool to generate the necessary input files for Paraview to visualize measured or simulated joint kinematics. 

## Simulation Description
A modified version of the lenhart2015 model with only the femur_distal_r and tibia_proximal_r bodies has been developed for this example (visualize_kinematics.osim)[./inputs/visualize_kinematics.osim].  The tibiofemoral coordinate values are defined vs time for each degree of freedom in the input_coordinate_values.sto. 

## Workflow Steps
1) Inspect the ./inputs/forsim_settings.xml file in a text editor to understand the property settings that setup the simulations.

Double click on run_anterior_laxity.cmd to perform the forsim simulation and joint-mechanics analysis for the healthy and ACL deficient models using the command line. You can open this file in a text editor to understand the code format to run the forsim and joint-mechanics executables.

Use Paraview and/or the OpenSim GUI to visualize the simulation results instructions.
