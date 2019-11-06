# OpenSim JAM
_A framework to simulate **J**oint and **A**rticular **M**echanics in OpenSim._

_Author: Colin Smith_

# This project is still in development. An official release will occur at the CAMS-Knee OpenSim Workshop at ETH Zürich in Feb 4-7 2019. Please be aware that naming conventions and structure may change until this date

This collection of OpenSim force component plugins, models, and executables (tools) are designed to enable simulations that include 6 degree of freedom joint mechanics with explicit representations of articular contact and ligament structures. It is currently a standalone package that requires OpenSim 4.0 to be installed. As development of pieces are finished I intend to integrate them into the OpenSim source code (opensim-core). 

Much of the included models and codes were initially developed in the [UW Neuromuscular Biomechanics Lab at the University of Wisconsin-Madison](https://uwnmbl.engr.wisc.edu). Contributers to this work include Darryl Thelen, Rachel Lenhart, Jarred Kaiser, Michael Vignos, Kwang won Choi, Scott Brandon, and Josh Roth. Translation and extension of the orginal SIMM and UWPipeline into OpenSim was performed during my time as a PhD at UW-Madison, NCSRR visiting scholar at Stanford, and as a post-doc at ETH Zürich. 

## Software:
### Dependencies
#### [OpenSim 4.0](https://opensim.stanford.edu/)
Needed for the executables to link against and to visualize models and simulation results.

#### [HDF5](https://www.hdfgroup.org/solutions/hdf5/)
HDF5 is a library used to generate .h5 files which store data in a binary hiearchial structure. This is used to save the per triangle contact distance and pressure maps in a compact format. MATLAB and Python both have readers available. This file type can be read much faster than text files, which is convinent when performing large scale monte carlo type analyses. The source code for HDF5 is redistributed as a dependency library within this repository (LINK). See source code or website for their licensing information.\ 



### Addtional Software
#### [HDFView](https://www.hdfgroup.org/downloads/hdfview/)
HDFviewer is a standalone gui to explore .h5 file contents. 

#### [Paraview 5.7](https://www.paraview.org/)
High quality rendering of simulation results, visualization of contact maps. 
 
#### [Mesh Mixer](http://www.meshmixer.com/) 
Nice software for mesh refinement, cutting, smoothing etc.

#### [Mesh Lab](http://www.meshlab.net/)

#### [GIBBON](https://www.gibboncode.org/)
MALTAB based software toolkit that includes mesh editing features. 

## OpenSim Components
For details of implementation and usage, see the doxygen links. 

### Blankevoort1991_Ligament
This one dimensional path geometry acts as a spring-damper to represent ligament fibers. The force strain relationship includes a nonlinear toe region at low strains and a linear region at high strains to represent the uncrimping and stretching of collagen fibers within a ligament. 
[Blakevoort1991_Ligament Doxygen]()

**Relavent Publications**:
*Smith, C. R., Lenhart, R. L., Kaiser, J., Vignos, M. F., & Thelen, D. G. (2016). Influence of ligament properties on tibiofemoral mechanics in walking. The journal of knee surgery, 29(02), 099-106.*

Smith, C. R., Vignos, M. F., Lenhart, R. L., Kaiser, J., & Thelen, D. G. (2016). The influence of component alignment and ligament properties on tibiofemoral contact forces in total knee replacement. Journal of biomechanical engineering, 138(2), 021017.
  
### Smith2018_ContactTriangleMesh
This component stores a triangular mesh representation of an articular surface that can be used in a Smith2018_ArticularContactForce contact pair. 
[Blakevoort1991_Ligament Doxygen]()
### Smith2018_ArticularContactForce
This force component uses an elastic foundation model to compute contact pressures on each contacting triangle on a pair of Smith2018_ContactTriangleMesh components. The pressure is computed as a function of the local depth of penetration. To visualize the pressure maps, you must use the joint-mechanics tool to generate .vtp files that can be read into Paraview. 
[Blakevoort1991_Ligament Doxygen]()

**Relavent Publications**:
*Smith, C. R., Won Choi, K., Negrut, D., & Thelen, D. G. (2018). Efficient computation of cartilage contact pressures within dynamic simulations of movement. Computer Methods in Biomechanics and Biomedical Engineering: Imaging & Visualization, 6(5), 491-498.*

*Bei, Y., & Fregly, B. J. (2004). Multibody dynamic simulation of knee contact mechanics. Medical engineering & physics, 26(9), 777-789.*

## Models
### Lenhart2015 
A multibody knee model was constructed based on magnetic resonance images from a healthy young adult female. The bones, ligaments, and cartilage were segmented from different image sets that were optimized for the discerning the relavent structures and registered. The tibiofemoral and patellofemoral joints are 

**Publications**
*Lenhart, R. L., Kaiser, J., Smith, C. R., & Thelen, D. G. (2015). Prediction and validation of load-dependent behavior of the tibiofemoral and patellofemoral joints during movement. Annals of biomedical engineering, 43(11), 2675-2685*

### Smith2018 
This model extends the Lenhart2015 model to include representations of the medial and lateral mensici. The menisici are connected via 6 degree of freedom joints to the tibia. Smith2018_ArticularContact is implemented 

**Examples**

**Publications**
*Smith, C. R., Brandon, S. C., & Thelen, D. G. (2019). Can altered neuromuscular coordination restore soft tissue loading patterns in anterior cruciate ligament and menisci deficient knees during walking?. Journal of biomechanics, 82, 124-133.*


## Simulation Tools
### Forsim 
Similar to the forward simulation tool in the OpenSim gui, but the interface is designed for performing forward simulations involving articular contact. The tool exposes the BDF implicit integrator, which gives far superior performance for simulations for contact. There are interfaces for prescribing joint kinematics and muscle forces to replicate cadaver experiments or passive experiments. 

### COMAK
The **C**oncurrent **O**ptimization of **M**uscle **A**ctivations and **K**inematics simulation tool enables the calculation of muscle forces and joint mechanics during dynamic movements. In early publications we also called this algorithim Enhanced Static Optimization before we came up with the clearly superior COMAK acroynm. 

Examples:

Publications:

### Joint Mechanics
This tool enables detailed post-hoc analysis of simulations involving joint mechanics. It can be used to generate .vtp files to visualize simulation results in Paraview, or .h5 files which are binary files that can store the large quantites of contact data (multiple calculated values for each triange face) in compact files that can be quickly read by MATLAB, Python, or HDF View (https://www.hdfgroup.org/downloads/hdfview/)

**Examples**
Visualize Measured Kinematics: 

**JAM References**
Below are all the references I am aware of that use these models or tools in some form. Note that all of these were performed using the older UWPipeline/SIMM implementation. Some of the early papers use a modified version of CMC to simulate the 6 DOF knee mechanics during walking (details in Thelen, JBME, 2014). This algorithm is not included in JAM and we don't plan to implement it at the moment.\ 

Clouthier, A., Borschneck, D., Thelen, D. G., Deluzio, K., & Rainbow, M. J. (2019). Relationship Between Lateral Patellar Stability Following Tibial Tubercle Osteotomy for Varying Patellofemoral Geometries. Journal of biomechanical engineering.

Meireles, S., Reeves, N. D., Jones, R. K., Smith, C. R., Thelen, D. G., & Jonkers, I. (2019). Patients with medial knee osteoarthritis reduce medial knee contact forces by altering trunk kinematics, progression speed, and stepping strategy during stair ascent and descent: a pilot study. Journal of applied biomechanics, 35(4), 280-289.

Rakhsha, M., Smith, C. R., Recuero, A. M., Brandon, S. C., Vignos, M. F., Thelen, D. G., & Negrut, D. (2019). Simulation of surface strain in tibiofemoral cartilage during walking for the prediction of collagen fibre orientation. Computer Methods in Biomechanics and Biomedical Engineering: Imaging & Visualization, 7(4), 396-405.

Van Rossom, S., Wesseling, M., Smith, C. R., Thelen, D. G., Vanwanseele, B., & Jonkers, I. (2019). The influence of knee joint geometry and alignment on the tibiofemoral load distribution: A computational study. The Knee.

Clouthier, A. L., Smith, C. R., Vignos, M. F., Thelen, D. G., Deluzio, K. J., & Rainbow, M. J. (2019). The effect of articular geometry features identified using statistical shape modelling on knee biomechanics. Medical engineering & physics, 66, 47-55.

Smith, C. R., Brandon, S. C., & Thelen, D. G. (2019). Can altered neuromuscular coordination restore soft tissue loading patterns in anterior cruciate ligament and menisci deficient knees during walking?. Journal of biomechanics, 82, 124-133.

Zevenbergen, L., Smith, C. R., Van Rossom, S., Thelen, D. G., Famaey, N., Vander Sloten, J., & Jonkers, I. (2018). Cartilage defect location and stiffness predispose the tibiofemoral joint to aberrant loading conditions during stance phase of gait. PloS one, 13(10), e0205842.

Smith, C. R., Won Choi, K., Negrut, D., & Thelen, D. G. (2018). Efficient computation of cartilage contact pressures within dynamic simulations of movement. Computer Methods in Biomechanics and Biomedical Engineering: Imaging & Visualization, 6(5), 491-498.

Bittmann, M. F., Lenhart, R. L., Schwartz, M. H., Novacheck, T. F., Hetzel, S., & Thelen, D. G. (2018). How does patellar tendon advancement alter the knee extensor mechanism in children treated for crouch gait?. Gait & posture, 64, 248-254.

Van Rossom, S., Smith, C. R., Thelen, D. G., Vanwanseele, B., Van Assche, D., & Jonkers, I. (2018). Knee joint loading in healthy adults during functional exercises: implications for rehabilitation guidelines. journal of orthopaedic & sports physical therapy, 48(3), 162-173.

Brandon, S. C., Thelen, D. G., Smith, C. R., Novacheck, T. F., Schwartz, M. H., & Lenhart, R. L. (2018). The coupled effects of crouch gait and patella alta on tibiofemoral and patellofemoral cartilage loading in children. Gait & posture, 60, 181-187.

Brandon, S. C., Smith, C. R., & Thelen, D. G. (2018). Simulation of soft tissue loading from observed movement dynamics. Handbook of Human Motion, 395-428.

Lenhart, R. L., Smith, C. R., Schwartz, M. H., Novacheck, T. F., & Thelen, D. G. (2017). The effect of distal femoral extension osteotomy on muscle lengths after surgery. Journal of children's orthopaedics, 11(6), 472-478.

Meireles, S., Wesseling, M., Smith, C. R., Thelen, D. G., Verschueren, S., & Jonkers, I. (2017). Medial knee loading is altered in subjects with early osteoarthritis during gait but not during step-up-and-over task. PloS one, 12(11), e0187583.

Lenhart, R. L., Brandon, S. C., Smith, C. R., Novacheck, T. F., Schwartz, M. H., & Thelen, D. G. (2017). Influence of patellar position on the knee extensor mechanism in normal and crouched walking. Journal of biomechanics, 51, 1-7.

Van Rossom, S., Smith, C. R., Zevenbergen, L., Thelen, D. G., Vanwanseele, B., Van Assche, D., & Jonkers, I. (2017). Knee cartilage thickness, T1ρ and T2 relaxation time are related to articular cartilage loading in healthy adults. PloS one, 12(1), e0170002.

Smith, C. R., Vignos, M. F., Lenhart, R. L., Kaiser, J., & Thelen, D. G. (2016). The influence of component alignment and ligament properties on tibiofemoral contact forces in total knee replacement. Journal of biomechanical engineering, 138(2), 021017.

Smith, C. R., Lenhart, R. L., Kaiser, J., Vignos, M. F., & Thelen, D. G. (2016). Influence of ligament properties on tibiofemoral mechanics in walking. The journal of knee surgery, 29(02), 099-106.

Lenhart, R. L., Kaiser, J., Smith, C. R., & Thelen, D. G. (2015). Prediction and validation of load-dependent behavior of the tibiofemoral and patellofemoral joints during movement. Annals of biomedical engineering, 43(11), 2675-2685.

Lenhart, R. L., Smith, C. R., Vignos, M. F., Kaiser, J., Heiderscheit, B. C., & Thelen, D. G. (2015). Influence of step rate and quadriceps load distribution on patellofemoral cartilage contact pressures during running. Journal of biomechanics, 48(11), 2871-2878.




