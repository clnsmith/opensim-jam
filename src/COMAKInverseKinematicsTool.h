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
 * This tool performs enables inverse kinematics to be performed for models 
 that include joints where some coordinates (degrees of freedom) can be 
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
    OpenSim_DECLARE_PROPERTY(model_file, std::string, "Location of model to use in COMAK simulation.")
    OpenSim_DECLARE_PROPERTY(results_dir, std::string, "Path to directory to write output results.")
    OpenSim_DECLARE_PROPERTY(results_prefix, std::string, "Prefix to all results files names.")
    OpenSim_DECLARE_PROPERTY(use_visualizer, bool, "Use SimTK visualizer to display simulations in progress.")


    OpenSim_DECLARE_PROPERTY(perform_secondary_constraint_sim, bool,
        "Perform forward simulation where secondary_coupled_coord is prescribed and secondardy coordinates are unconstrained to find constraint functions for IK.")
    OpenSim_DECLARE_LIST_PROPERTY(secondary_coordinates, std::string, "List the Secondary Coordinates in the model.")
    OpenSim_DECLARE_OPTIONAL_PROPERTY(secondary_coupled_coord, std::string, "Name of coordinate to prescribe in simulation.")
    OpenSim_DECLARE_PROPERTY(secondary_constraint_sim_settle_time, double, "Initial settle period where secondary_coupled_coord is locked and secondary coords settle.")
    OpenSim_DECLARE_PROPERTY(secondary_coupled_coord_start_value, double, "Initial value for secondary_coupled_coord. In degrees for rotational coordinates.")
    OpenSim_DECLARE_PROPERTY(secondary_coupled_coord_stop_value, double, "Final value for secondary_coupled_coord. In degrees for rotational coordinates.")
    OpenSim_DECLARE_PROPERTY(secondary_constraint_sim_time, double, "Length of time to linearly prescribe secondary_couple_coord through the start and end value.")
    OpenSim_DECLARE_PROPERTY(secondary_constraint_sim_integrator_accuracy, double, "Integrator tolerance for forward simulation.")
    OpenSim_DECLARE_PROPERTY(secondary_constraint_sim_damping_multiplier, double, "Integrator tolerance for forward simulation.")
    OpenSim_DECLARE_PROPERTY(print_secondary_constraint_sim_results, bool, "Print results .sto file for secondary_constraint_sim")
    OpenSim_DECLARE_PROPERTY(secondary_constraint_function_file, std::string, "Path to file where secondary constraint functions are saved.")

    OpenSim_DECLARE_PROPERTY(perform_inverse_kinematics, bool, "Perform Inverse Kinematics")
    OpenSim_DECLARE_PROPERTY(inverse_kinematics_tool, InverseKinematicsTool, "InverseKinematicsTool that calculates the joint kinematics after secondary constraints are added to model.")

    OpenSim_DECLARE_OPTIONAL_PROPERTY(ik_print_model_file, std::string, "Path to output .osim file.")
    OpenSim_DECLARE_PROPERTY(verbose, int, "Level of debug information reported (0: low, 1: medium, 2: high)")



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


