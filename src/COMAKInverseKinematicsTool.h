#ifndef OPENSIM_COMAK_INVERSE_KINEMATICS_TOOL_H_
#define OPENSIM_COMAK_INVERSE_KINEMATICS_TOOL_H_
/* -------------------------------------------------------------------------- *
 *                      COMAKInverseKinematicsTool.h                          *
 * -------------------------------------------------------------------------- *
 * Author(s): Colin Smith                                                     *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */


#include <OpenSim\Simulation\Model\Model.h>
#include <OpenSim/Common/FunctionSet.h>
#include <OpenSim/Tools/InverseKinematicsTool.h>
#include "osimPluginDLL.h"

namespace OpenSim { 

//=============================================================================
//=============================================================================
/**
 This tool enables inverse kinematics to be performed for models 
 that include joints where some Coordinates (degrees of freedom) can be 
 accurately determined from motion capture (marker_determined) and others 
 cannot (secondary). A forward dynamic simulation is performed to obstrain a 
 set of constraint functions to couple the secondary coordinates to specific 
marker determined coordinates.  
 *
 * @author Colin Smith

 */
class OSIMPLUGIN_API COMAKInverseKinematicsTool : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(COMAKInverseKinematicsTool, Object)

public:
    OpenSim_DECLARE_PROPERTY(model_file, std::string, 
        "Path to .osim model file.")

    OpenSim_DECLARE_PROPERTY(results_directory, std::string,
        "Path to directory to write output results.")

    OpenSim_DECLARE_PROPERTY(results_prefix, std::string, 
        "Prefix to all results files names.")

    OpenSim_DECLARE_PROPERTY(perform_secondary_constraint_sim, bool,
        "Perform forward simulation where secondary_coupled_coordinate is "
        "prescribed and secondardy coordinates are unconstrained to "
        "generate the coupled constraint functions for the "
        "seconday coordinates for inverse kinematics."
        "The default value is true.")
 
    OpenSim_DECLARE_LIST_PROPERTY(secondary_coordinates, std::string, 
        "List of paths to the Secondary Coordinates in the model.")

    OpenSim_DECLARE_OPTIONAL_PROPERTY(secondary_coupled_coordinate, std::string, 
        "Path to the coordinate to prescribe in "
        "secondary_constraint_simulation. In inverse kinematics, "
        "secondary_coupled_coord will be used as the "
        "independent_coordinate_name for the CoordinateCouplerConstraints "
        "for all secondary coordinates. ")

    OpenSim_DECLARE_PROPERTY(secondary_constraint_sim_settle_time, double, 
        "Duration of the initial settle period where the "
        "secondary_coupled_coordinate is locked and secondary coordinates "
        "settle into equilibrium."
        "The default value is 1.0 seconds.")

    OpenSim_DECLARE_PROPERTY(secondary_constraint_sim_sweep_time, double, 
        "The duration of the simulation phase where the "
        "secondary_coupled_coord is swept (linearly prescribed) through the "
        "range of motion from the  secondary_coupled_coordinate_start_value "
        "to the secondary_coupled_coordinate_stop_value.")

    OpenSim_DECLARE_PROPERTY(secondary_coupled_coordinate_start_value, double, 
        "Initial Coordinate value for the secondary_coupled_coordinate in the "
        "secondary_constraint_sim. The units are in meters for translational "
        "coordinates and degrees for rotational coordinates. "
        "The default value is 0.0.")

    OpenSim_DECLARE_PROPERTY(secondary_coupled_coordinate_stop_value, double, 
        "Initial Coordinate value for the secondary_coupled_coordinate in the "
        "secondary_constraint_sim. The units are in meters for translational "
        "coordinates and degrees for rotational coordinates. "
        "The default value is 0.0.")

    OpenSim_DECLARE_PROPERTY(
        secondary_constraint_sim_integrator_accuracy, double, 
        "Integrator tolerance for the forward simulation."
        "The default value is 1e-6.")

    OpenSim_DECLARE_PROPERTY(
        secondary_constraint_sim_internal_step_limit, int,
        "Limit on the number of internal steps that can be taken by BDF "
        "integrator. If -1, then there is no limit. The Default value is -1")

     OpenSim_DECLARE_PROPERTY(secondary_constraint_function_file, std::string, 
        "Name for .xml results file where secondary constraint functions "
        "will be saved. "
        "The default value is "
        "'secondary_coordinate_constraint_functions.xml'.")

    OpenSim_DECLARE_PROPERTY(print_secondary_constraint_sim_results, bool, 
        "Print model states to a .sto file for secondary_constraint_sim. "
        "The default value is false.")

    OpenSim_DECLARE_PROPERTY(perform_inverse_kinematics, bool, 
        "Perform Inverse Kinematics where CoordinateCouplerConstraints are "
        "added to the model where the secondary_coordinates are coupled to "
        "the secondary_coupled_coordinate. "
        "The default value is true.")

    OpenSim_DECLARE_UNNAMED_PROPERTY(InverseKinematicsTool, 
        "The InverseKinematicsTool that is used to calculate the joint "
        "kinematics after CoordinateCouplerConstraints for the secondary "
        "constraints are added to model.")

    OpenSim_DECLARE_PROPERTY(constrained_model_file, std::string, 
        "Print the .osim model file with added CoordinateCouplerConstraints "
        "that is used in the InverseKinematicsTool. If empty, no model file "
        "will be printed.")
    
    OpenSim_DECLARE_PROPERTY(use_visualizer, bool,
        "Use SimTK visualizer to display the model during "
        "secondary_constraint_sim and inverse kinematics."
        "The default value is false.")

    OpenSim_DECLARE_PROPERTY(verbose, int, 
        "Level of debug information written to the console. "
        "(0: silent). "
        "The default value is 0.")



    //=============================================================================
// METHODS
//=============================================================================    
  
    /**
    * Default constructor.
    */
    COMAKInverseKinematicsTool();
    
    //Construct from .xml file
    COMAKInverseKinematicsTool(const std::string file);

private:
    void constructProperties();

public:
    void initialize();
    void run();
    void performIKSecondaryConstraintSimulation();
    void performIK();
    //--------------------------------------------------------------------------
    // Members
    //--------------------------------------------------------------------------
public:
    Model _model;
    SimTK::State _state;

    int _n_prescribed_coord;
    int _n_primary_coord;
    int _n_secondary_coord;

    int _n_muscles;
    int _n_reserve_actuators;
    int _n_actuators;
    int _n_parameters;
    Array<std::string> _parameter_names;

    Array<std::string> _prescribed_coord_name;
    Array<std::string> _prescribed_coord_path;
    Array<int> _prescribed_coord_index;

    Array<std::string> _primary_coord_name;
    Array<std::string> _primary_coord_path;
    Array<int> _primary_coord_index;

    Array<std::string> _secondary_coord_name;
    Array<std::string> _secondary_coord_path;
    Array<int> _secondary_coord_index;

    int _n_frames;
    int _n_out_frames;
    int _start_frame;
    Array<double> _time;
    double _dt;
    int _consecutive_bad_frame;
    std::vector<int> _bad_frames;
    std::vector<double> _bad_times;
    std::vector<double> _bad_udot_errors;
    std::vector<std::string> _bad_udot_coord;

    SimTK::Matrix _q_matrix;
    SimTK::Matrix _u_matrix;
    SimTK::Matrix _udot_matrix;
    ExternalLoads _external_loads;

    SimTK::Vector _secondary_coord_damping;
    SimTK::Vector _optimal_force;
    SimTK::Vector _prev_secondary_value;
    SimTK::Vector _prev_parameters;
    SimTK::Vector _parameter_scale;
    SimTK::Vector _muscle_volumes;

    FunctionSet _secondary_constraint_functions;
    std::string _directoryOfSetupFile;
//=============================================================================
};  // END of class COMAK_INVERSE_KINEMATICS_TOOL

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_COMAK_INVERSE_KINEMATICS_TOOL_H_


