# OpenSim JAM

<img  height="300" src="graphics/msk_model_walk_full_body.png"><img  height="300" src="graphics/tka_ligament_elongation_walking.gif"><img  height="300" src="graphics/tka_contact.png">

_A framework to simulate **J**oint and **A**rticular **M**echanics in OpenSim._

_Author: Colin Smith_

This repository is a collection of force component plugins, models, and executables (tools) that are designed to enable OpenSim musculoskeletal simulations that detailed joint mechanics. This includes 6 degree of freedom joint representations (without kinematic constraints) that use explicit representations of articular contact and ligament structures. 

This repository is used for the development of these tools and contains the [source code](src) for this package. Instructions to build the C++ code from source are provided [here](./documentation/building-opensim-jam-from-source.md).

A release version of these codes is provided in the [opensim-jam-release](opensim-jam-release) folder. This includes prebuilt versions of the opensim_jam.dll plugin for OpenSim, command-line exectuables for the tools, examples, and documentation. 
