# OpenSim JAM
_A framework to simulate **J**oint and **A**rticular **M**echanics in OpenSim._

_Author: Colin Smith_

Much of the included models and codes were initially developed in the UW Neuromuscular Biomechanics Lab at the University of Wisconsin-Madison. Contributers to this work include Darryl Thelen, Rachel Lenhart, Jarred Kaiser, Michael Vignos, Kwang won Choi, Scott Brandon, and Josh Roth. Translation and extension of the orginal SIMM and UWPipeline into OpenSim was performed during my time as a PhD at UW-Madison, NCSRR visiting scholar at Stanford, and as a post-doc at ETH Zurich. 

## Force Components
### Blankevoort1991_Ligament
This one dimensional path geometry acts as a spring-damper to represent ligament fibers. The force strain relationship includes a nonlinear toe region at low strains and a linear region at high strains to represent the uncrimping and stretching of collagen fibers within a ligament. Details of the ligament implementation can be found in the doxygen file.<LINK TO DOXY> 

**Examples**:

**Publications**:
*Smith, C. R., Lenhart, R. L., Kaiser, J., Vignos, M. F., & Thelen, D. G. (2016). Influence of ligament properties on tibiofemoral mechanics in walking. The journal of knee surgery, 29(02), 099-106.*

Smith, C. R., Vignos, M. F., Lenhart, R. L., Kaiser, J., & Thelen, D. G. (2016). The influence of component alignment and ligament properties on tibiofemoral contact forces in total knee replacement. Journal of biomechanical engineering, 138(2), 021017.
  
**Smith2018_ContactTriangleMesh**
This component stores the triangular mesh file that represents one of the articular surfaces in the 

**Smith2018_ArticularContactForce**
This force component uses an elastic foundation model to compute contact pressures on each contacting triangle on a pair of Smith2018_ContactTriangleMesh components. The pressure is computed as a function of the local depth of penetration. For implementation details see the doxygen file ***LINK***. To visualize the pressure maps, you must use the joint-mechanics tool to generate .vtp files that can be read into Paraview. 

*Smith, C. R., Won Choi, K., Negrut, D., & Thelen, D. G. (2018). Efficient computation of cartilage contact pressures within dynamic simulations of movement. Computer Methods in Biomechanics and Biomedical Engineering: Imaging & Visualization, 6(5), 491-498.*

Bei, Y., & Fregly, B. J. (2004). Multibody dynamic simulation of knee contact mechanics. Medical engineering & physics, 26(9), 777-789.

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



