# Example: Isometric Extension
In an isometric extention test, a clinician positions the hip and knee at 30<sup>o</sup> flexion and holds the thigh and ankle in place while the patient activates their quadriceps.

[<img src="https://img.youtube.com/vi/F7LRQARxCpQ/0.jpg" align="middle" height="300" >](https://youtu.be/F7LRQARxCpQ?t=60)



## Simulation Description
A forward simulation will be performed using the lenhart2015 model and forsim tool to predict the knee kinematics, and ligament and articular contact loading during the isometric extension clinical examination.

### Coordinates
The hip_flex_r and knee_flex_r coordinates are prescribed to flex from 0<sup>o</sup> to 30<sup>o</sup> and are then held constant. Once the peak flexion angles are reached, the unconstrained coordinates are allowed to settle into equilibrium and then the quadriceps are activated. 

_Unconstrained Coordinates_
- knee_add_r
- knee_rot_r
- knee_tx_r
- knee_ty_r
- knee_tz_r
- pf_flex_r
- pf_rot_r
- pf_tilt_r
- pf_tx_r
- pf_ty_r
- pf_tz_r

_Prescribed Coordinates_
- hip_flex_r
- knee_flex_r

_Fixed Coordinates_
- All others

### Muscles 
After the hip and knee are flexed, the quadriceps muscles are loaded. To demonstrate the capabilities of the forsim tool, the excitation, activation, and force of one each of the vastii muscles are prescribed to increase linearly. The activation of all muscles that are not listed below is set to the default minimum value, thus they only contribute passive forces to the simulation.

_Excitation Prescribed (0 to 0.5)_
- vasmed_r (vastus medialis)

_Activation Prescribed (0 to 0.5)_
- vaslat_r (vastus lateralis)

_Force Prescribed (0N to 25N)_ 
- vasint_r (vastus intermedius)

## Workflow Steps
1) Use MATLAB to run the [./inputs/generate_isometric_extension_input_files](./inputs/generate_isometric_extension_input_files) script to generate the [./inputs/prescribed_coordinates.sto](./inputs/prescribed_coordinates.sto) and [./inputs/prescribed_muscles.sto](./inputs/prescribed_muscles.sto) input files.

2) Inspect the [./inputs/forsim_settings.xml](./inputs/forsim_settings.xml) and [./inputs/joint_mechanics_settings.xml](./inputs/joint_mechanics_settings.xml) files

