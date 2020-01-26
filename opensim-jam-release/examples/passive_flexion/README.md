# Example: Passive Flexion
<p align="center">
  <img src="graphics/passive_flexion.gif" height="300" >
</p>


This example simulates passive flexion of the knee from 0<sup>o</sup> to 120<sup>o</sup>. The simulation is designed to replicate a clinician manually flexing a patient's knee or a robot manipulating a cadaver knee. Thus, the muscle activation is assumed to be minimal and only the passive muscle, ligament, and contact forces contribute to the predicted knee motion and loading. 

## Simulation Description
This simulation uses the [lenhart2015 model](../../models/lenhart2015/lenhart2015.osim). Tibiofemoral flexion (knee_flex_r) is prescribed using the prescribed_coordinates_file. Initially, knee_flex_r is held constant at 0<sup>o</sup> for 0.5 seconds to allow the knee to settle into equilibrium, then over the next 2.0 seconds knee_flex_r is linearly prescribed from 0<sup>o</sup> to 120<sup>o</sup>. All other knee Coordinates (5 tibiofemoral DOFs and 6 patellofemoral DOFs) are unlocked and thus their kinematics over the simulation are predicted based on the passive model forces. The pelvis_tilt Coordinate is prescribed to be 90<sup>o</sup> over the duration of the simulation to represent the patient lying on a table. The remaining Coordinates in the model are locked, and thus fixed at their default values throughout the simulation. 

## Workflow steps 
1) Use MATLAB to run the [./inputs/generate_passive_flexion_input_files.m](inputs/generate_passive_flexion_input_files.m) to generate the prescribed_coordinates.sto file containing the knee_flex_r and pelvis_tilt Coordinate values vs time. 

2) Inspect the [./inputs/forsim_settings.xml](inputs/forsim_settings.xml) and [./inputs/joint_mechanics_settings.xml](inputs/joint_mechanics_settings.xml) files in a text editor to understand the property settings used to setup the simulation.

3) Double click on [run_passive_flexion.cmd](./inputs/run_passive_flexion.cmd) to perform the forsim simulation and joint-mechanics analysis using the command line. You can open this file in a text editor to understand the code format to run the forsim and joint-mechanics executables.

4) Use Paraview and/or the OpenSim GUI to visualize the simulation results [instructions](../../documentation/visualizing-simulation-results).

5) Use MATLAB to run the [analyze_passive_flexion.m](analyze_passive_flexion.m) script to generate plots of knee kinematics and ligament strains vs time.  

