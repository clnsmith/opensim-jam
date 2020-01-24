# OpenSim-JAM
_A framework to simulate **J**oint and **A**rticular **M**echanics in OpenSim._

_Author: Colin Smith_

## Overview
This is a collection of force component plugins, models, and executables (tools) are designed to enable OpenSim musculoskeletal simulations that include 6 degree of freedom joint mechanics with explicit representations of articular contact and ligament structures.  This includes examples for visualizing measured kinematics, as well as simulating clinical evaluations, cadaver experiments, and functional movements such as walking. 

### Blankevoort1991Ligament
This one dimensional path geometry acts as a spring-damper to represent ligament fibers. The force strain relationship includes a nonlinear toe region at low strains and a linear region at high strains to represent the uncrimping and stretching of collagen fibers within a ligament. [Blakevoort1991Ligament Description](documentation/doxygen/Blakevoort1991Ligament_doxygen.pdf)

### Smith2018ArticularContactForce and Smith2018_ContactMesh
This force component models articular contact between cartilage, mensici, or artifical components using triangular mesh representations of the surface geometries and an elastic foundation model to compute local contact pressures.  
[Smith2018ArticularContactForce Description](documentation/doxygen/Smith2018ArticularContactForce_doxygen.pdf)

This component stores the material properties and triangular mesh geometries (.vtp, .stl, or .obj) of an articular surface used in a Smith2018_ArticularContactForce contact pair. 
[Smith2018ContactMesh_Description](documentation/doxygen/Smith2018ContactMesh_doxygen.pdf)

### Lenhart2015 Model
*Lenhart, R. L., Kaiser, J., Smith, C. R., & Thelen, D. G. (2015). Prediction and validation of load-dependent behavior of the tibiofemoral and patellofemoral joints during movement. Annals of biomedical engineering, 43(11), 2675-2685*


<img src="https://github.com/clnsmith/opensim-jam/blob/master/graphics/lenhart2015_fullbody.JPG" height="400"><img src="https://github.com/clnsmith/opensim-jam/blob/master/graphics/lenhart2015_knee.JPG" height="400">

A multibody knee model was constructed based on magnetic resonance images from a healthy young adult female. The bones, ligaments, and cartilage were segmented from different image sets that were optimized for the discerning the relavent structures and registered. The tibiofemoral and patellofemoral joints are 6 DOF joints. See publication for details of model construction. 

### Smith2018 Model (Coming Soon)
*Smith, C. R., Brandon, S. C., & Thelen, D. G. (2019). Can altered neuromuscular coordination restore soft tissue loading patterns in anterior cruciate ligament and menisci deficient knees during walking?. Journal of biomechanics, 82, 124-133.*

This model is based on the same subject and experimental data as the Lenhart2015 model, but also includes representations of the medial and lateral mensici. The menisici are connected via 6 degree of freedom joints to the tibia. Smith2018_ArticularContact is implemented beween the mensici and cartilage surfaces. 

## Simulation Tools
### Forsim 
Similar to the forward simulation tool in the OpenSim gui, but the interface is designed for performing forward simulations involving articular contact. The tool exposes the BDF implicit integrator, which gives far superior performance for simulations for contact. There are interfaces for prescribing joint kinematics and muscle forces to replicate cadaver experiments or passive experiments. 

### COMAK
The **C**oncurrent **O**ptimization of **M**uscle **A**ctivations and **K**inematics simulation tool enables the calculation of muscle forces and joint mechanics during dynamic movements. In early publications we also called this algorithim Enhanced Static Optimization before we came up with the clearly superior COMAK acronym. 

### Joint Mechanics
This tool enables detailed post-hoc analysis of simulations involving joint mechanics. It can be used to generate .vtp files to visualize simulation results in Paraview, or .h5 files which are binary files that can store the large quantites of contact data (multiple calculated values for each triange face) in compact files that can be quickly read by MATLAB, Python, or HDF View (https://www.hdfgroup.org/downloads/hdfview/)

## JAM Distribution
The opensim-jam-distribute folder contains everything you need to get started using the OpenSim components and applications (tools) described above. 

The jam-plugin.dll is compilied for you and located [here](opensim-jam-distribute/bin/jam-plugin.dll)

### Examples

### [Visualize Kinematics](opensim-jam-distribute/examples/visualize_kinematics)
You can use the joint-mechanics tool to analyze and visualize simulated or measured bone kinematics from fluoroscopy, dynamic MRI, etc or static poses from medical imaging. The tool can calculate distance maps between bone surfaces or overlap maps if cartilage surfaces are also provided.

### [Passive Flexion](opensim-jam-distribute/examples/passive_flexion)
Perform a forward simulation of passive knee flexion where tibiofemoral flexion is prescribed and the other knee coordinates are unconstrained and thus are calculated based on the passive muscle, ligament, and articular contact forces. 

### [Anterior Laxity](opensim-jam-distribute/examples/anterior_laxity)
Replicate a clinical test for anterior cruciate ligament (ACL) deficiency by performing a forward simulation where the hip is flexed to 45<sup>o</sup>, the knee is flexed to 90<sup>o</sup> and an anterior force is applied to the tibia.

### [Isometric Extension](opensim-jam-distribute/examples/isometric_extension)
Replicate a clinical isometric extension test where the hip and knee are held at 30<sup>o</sup> flexion and the patient activates their quadriceps. 

### [Walking](opensim-jam-distribute/examples/walking)
Use the COMAK tool to predict muscle forces, ligament forces, cartilage contact pressures and secondary knee kinematics during walking. 

## Software:

### [OpenSim 4.0 GUI](https://opensim.stanford.edu/)
The OpenSim GUI is helpful for visualizing models and simulation results. The tool executables 

### MATLAB

### [Paraview 5.7](https://www.paraview.org/)
High quality rendering of simulation results, visualization of contact maps. 
[Vizualizing in Paraview](documentation/visualizing_in_paraview.md)

### [HDFView](https://www.hdfgroup.org/downloads/hdfview/)
HDFviewer is a standalone gui to explore .h5 file contents. 

### [Mesh Mixer](http://www.meshmixer.com/) 
Software for mesh visualization, refinement, cutting, smoothing etc.

### [Mesh Lab](http://www.meshlab.net/)
Software for mesh visualization, smoothing etc. 

### [GIBBON](https://www.gibboncode.org/)
MATLAB based software toolkit that includes mesh editing features.


## Acknowledgements 
Much of the included models and codes were initially developed in the [UW Neuromuscular Biomechanics Lab at the University of Wisconsin-Madison](https://uwnmbl.engr.wisc.edu). Contributers to this work include Darryl Thelen, Rachel Lenhart, Jarred Kaiser, Michael Vignos, Kwang won Choi, Scott Brandon, and Josh Roth. Translation and extension of the orginal SIMM and UWPipeline into OpenSim was performed during my time as a PhD at UW-Madison (NIH EB015410) and Stanford ([NCSRR Visiting Scholar](https://opensim.stanford.edu/support/scholars.html)), and as a post-doc at the [Laboratory for Movement Biomechanics](https://www.movement.ethz.ch/) at ETH Zürich ([Whitaker International Program](https://www.whitaker.org/), [OpenSim Pilot Project](https://opensim.stanford.edu/support/pilot.html)).
