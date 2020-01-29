# Example: Visualize Kinematics
<p align="center">
  <img src="graphics/visualize_kinematics.gif" height="300" >
</p>

This example demonstrates the ability of the JointMechanicsTool to generate the necessary input files for Paraview to visualize measured or simulated joint kinematics. Here, "measured" tibiofemoral kinematics are used to pose the bones over a walking cycle, and the proximity between the femoral and tibial subchondral bone surfaces are mapped. These bone kinematics could be measured from fluoroscopy, dynamic MRI etc (the example kinematics are actually simulation results). Additionally, the length change of ligament bundles representing the MCL are  visualized.    

## Simulation Description
The [visualize_kinematics.osim](./inputs/visualize_kinematics.osim) model used in the simulation is a modified version of the lenhart2015 model with only the femur_distal_r and tibia_proximal_r bodies. The included [lenhart2015-R-femur-subchondral-bone.stl](./inputs/Geometry/lenhart2015-R-femur-subchondral-bone.stl) and [lenhart2015-R-tibia-subchondral-bone.stl](./inputs/Geometry/lenhart2015-R-tibia-subchondral-bone.stl) meshes were created by manually selecting a region of the bone meshes that approximates the subchondral bone surface. These meshes are used in the Smith2018ArticularContactForce component to calculate the proximity maps. Three Blankevoort1991Ligament components were placed on the model in order to represent the anterior, mid, and posterior regions of the MCL. 

The tibiofemoral coordinate values are defined vs time for each degree of freedom in the [input_coordinate_values.sto](./inputs/input_coordinate_values.sto) file. These are input to the JointMechanicsTool to generate the .vtp files for visualization in Paraview.    

## Workflow Steps
1) Inspect the [joint_mechanics_settings.xml](./inputs/joint_mechanics_settings.xml) file in a text editor to understand the property settings that setup the simulations.

2) Double click on [run_visualize_kinematics.cmd](run_visualize_kinematics.cmd) to use the windows command line to run the JointMechanicsTool. You can open this file in a text editor to understand the code syntax to run the joint-mechanics executables.

3) Use Paraview and/or the OpenSim GUI to visualize the simulation results [instructions](../../documentation/visualizing-models-and-simulation-results.md).
