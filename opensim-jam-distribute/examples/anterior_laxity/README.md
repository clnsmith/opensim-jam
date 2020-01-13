# Example: Anterior Laxity
[<img src="https://pbs.twimg.com/media/DX1r43jX4AAl0dk?format=jpg&name=small" width="400">](https://twitter.com/healthcrib_fdt/status/972051732414033920)

[<img src="https://img.youtube.com/vi/IdnBKv38EEQ/0.jpg" width="400">](https://www.youtube.com/watch?v=IdnBKv38EEQ&t=7s)

## Simulation Description
This simulation replicates an anterior laxity test to diagnose ACL deficiency. The hip_flex_r coordinate is prescribed to flex from 0<sup>o</sup> to 45<sup>o</sup>, and the knee_flex_r coordinate is prescribed to flex from 0<sup>o</sup> to 90<sup>o</sup>. Once peak flexion is reached, the hip and knee flexion are prescribed to be constant and a 100 N anterior force is applied to the tibia. The muscles are minimally activated (0.02) and thus only provide passive forces. The remaining knee coordinates (5 tibiofemoral DOFs and 6 patellofemoral DOFs) are unconstrained and thus their kinematics are predicted as a result of the muscle, ligament, and cartilage contact forces. 

## Workflow Steps
1) Run the [./inputs/generate_ie_laxity_input_files.m](inputs/generate_ie_laxity_input_files.m) script in MATLAB to generate the prescribed_coordinates.sto file containing the hip_flex_r and knee_flex_r values vs time, a external_loads.sto file containing the anterior force values vs time, and a external_loads.xml file describing how the anterior force is applied to the model. 

2) Inspect the [./inputs/forsim_settings.xml](inputs/forsim_settings.xml) and [./inputs/joint_mechanics_settings.xml](inputs/joint_mechanics_settings.xml) files in a text editor to understand the setup for the simulations

3) Double click on [run_anterior_laxity.cmd](./inputs/joint_mechanics_settings.xml) to perform the forsim simulation and joint-mechanics analysis using the command line. 

4) Use paraview to visualize the simulation results.

5) Use MATLAB to run the analyze_anterior_laxity.m script to generate plots of anterior translation and ligament forces vs time.  
