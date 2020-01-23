# Example: Walking

## Simulation Description
This example uses COMAK to predict muscle forces, ligament forces, cartilage contact pressures, and secondary knee kinematics (5 tibiofemoral DOFs, 5 patellofemoral DOFs) during walking. 

The experimental data (motion capture, ground reaction forces) were collected while the Lenhart2015 subject performed overground walking at a self selected speed. 


## Workflow Steps
1) Inspect the [./inputs/comak_inverse_kinematics_settings.xml](./inputs/comak_inverse_kinematics_settings.xml) and [./inputs/comak_settings.xml](./inputs/comak_settings.xml) files to understand the inputs and settings for the simulation.

2) Inspect the [run_walking.cmd](run_walking.cmd) file.

3) Double click on the [run_walking.cmd](run_walking.cmd) file to perform the simulation.

4) Visualize the results in Paraview.

5) Use MATLAB to run analyze_walking_results.m to generate plots of the simulation results. 
