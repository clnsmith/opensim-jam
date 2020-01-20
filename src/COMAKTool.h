#ifndef OPENSIM_COMAK_TOOL_H_
#define OPENSIM_COMAK_TOOL_H_
/* -------------------------------------------------------------------------- *
 *                                 COMAKTool.h                                *
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
#include <OpenSim/Simulation/Model/ExternalLoads.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include "osimPluginDLL.h"
#include <OpenSim/Simulation/Model/AnalysisSet.h>

namespace OpenSim { 
class COMAKSecondaryCoordinate;
class COMAKSecondaryCoordinateSet;
class COMAKCostFunctionParameter;
class COMAKCostFunctionParameterSet;

class OSIMPLUGIN_API COMAKTool : public Object{
    OpenSim_DECLARE_CONCRETE_OBJECT(COMAKTool, Object)

public:
    OpenSim_DECLARE_PROPERTY(model_file, std::string, 
        "Path to .osim model to use in COMAK simulation.")
    OpenSim_DECLARE_PROPERTY(ik_motion_file, std::string, 
        "Path to input .mot file containing joint angles vs time for all "
        "prescribed and primary coordinates. ")
    OpenSim_DECLARE_PROPERTY(external_loads_file, std::string, "Path to .xml "
        "file that defines the ExternalLoads applied to the model.")
    OpenSim_DECLARE_PROPERTY(results_dir, std::string, 
        "Path to directory to write results files.")
    OpenSim_DECLARE_PROPERTY(results_prefix, std::string, 
        "Prefix to all results files names.")
    OpenSim_DECLARE_PROPERTY(replace_force_set, bool, 
        "Replace the model ForceSet with the forces listed in force_set_file."
        " If false, force_set_file forces are appended to the existing model "
        "force set. The default value is false.")
    OpenSim_DECLARE_PROPERTY(force_set_file,std::string,
        "Path to .xml file containing an additional ForceSet.")

    OpenSim_DECLARE_PROPERTY(start_time, double, 
        "First time step of COMAK simulation.")
    OpenSim_DECLARE_PROPERTY(stop_time, double, 
        "Last time step of COMAK simulation.")
    OpenSim_DECLARE_PROPERTY(time_step, double,
        "Time increment between steps in COMAK simulation.Set to -1 to use "
        "input kinematics time step.")
    OpenSim_DECLARE_PROPERTY(lowpass_filter_frequency, double, 
        "Lowpass filter frequency for input kinematics. "
        "Set to -1 to use raw data")
    OpenSim_DECLARE_PROPERTY(print_input_kinematics, bool, "Print processed input q's, u's, and udots.")

    OpenSim_DECLARE_LIST_PROPERTY(prescribed_coordinates, std::string, "List the Prescribed Coordinates in the model.")
    OpenSim_DECLARE_LIST_PROPERTY(primary_coordinates, std::string, "List the Primary Coordinates in the model.")
    OpenSim_DECLARE_UNNAMED_PROPERTY(COMAKSecondaryCoordinateSet,"List of COMAKSecondaryCoodinate objects.")

    OpenSim_DECLARE_PROPERTY(equilibriate_secondary_coordinates_at_start, bool, "Perform a forward simulation to settle secondary coordinates into equilbrium at initial time step of COMAK.")
    OpenSim_DECLARE_PROPERTY(settle_accuracy, double, "Integrator accuracy for initializing forward simulation.")
    OpenSim_DECLARE_PROPERTY(settle_threshold, double, "Maximum change in secondary coordinates between timesteps that defines equilibrium.")
    OpenSim_DECLARE_PROPERTY(print_settle_sim_results, bool, "Print the coordinate and speed values during the forward simulation to settle_sim_results_dir.")
    OpenSim_DECLARE_PROPERTY(settle_sim_results_prefix, std::string, "Prefix to settle simulation results file names.")
    OpenSim_DECLARE_PROPERTY(settle_sim_results_dir, std::string, "Path to results directory to print forward simulation results.")

    OpenSim_DECLARE_PROPERTY(max_iterations, int, "Max number of iterations per time step for the COMAK optimization.")
    OpenSim_DECLARE_PROPERTY(max_change_rotation, double, "Maximum change in rotational secondary coordinates between COMAK iterations")
    OpenSim_DECLARE_PROPERTY(max_change_translation, double, "Maximum change in translational secondary coordinates between COMAK iterations")
    OpenSim_DECLARE_PROPERTY(udot_tolerance, double, "Tolerance on achieving the desired udots.")
    OpenSim_DECLARE_PROPERTY(udot_worse_case_tolerance, double, "Max error in udot acceptable if no iterations converge, if max udot error > worse case tolerance the previous time step is used.")
    OpenSim_DECLARE_PROPERTY(unit_udot_epsilon, double, "Perturbation applied to secondary_coords when computing the linearized version of the COMAK acceleration constraints.")    
    OpenSim_DECLARE_PROPERTY(contact_energy_weight, double, "Weighting term for contact energy in COMAK cost function")
    OpenSim_DECLARE_PROPERTY(verbose, int, "Level of debug information reported (0: low, 1: medium, 2: high)")
    OpenSim_DECLARE_PROPERTY(use_visualizer, bool, "Use SimTK visualizer to display simulations in progress.")

    OpenSim_DECLARE_UNNAMED_PROPERTY(COMAKCostFunctionParameterSet,"List of COMAKCostFunctionWeight objects.")
    OpenSim_DECLARE_UNNAMED_PROPERTY(AnalysisSet,"Analyses to be performed"
		"during forward simulation.")
//=============================================================================
// METHODS
//=============================================================================
  
    /**
    * Default constructor.
    */
    COMAKTool();
    
    //Construct from .xml file
    COMAKTool(const std::string file);

private:
    void constructProperties();
    void updateModelForces();
    void initialize();
    void extractKinematicsFromFile();
    void applyExternalLoads();
    void printCOMAKascii();
    SimTK::Vector equilibriateSecondaryCoordinates();
    void performCOMAK();
    void setStateFromComakParameters(SimTK::State& state, const SimTK::Vector& parameters);
    SimTK::Vector computeMuscleVolumes();
    void printOptimizationResultsToConsole(const SimTK::Vector& parameters);
public:
    void run();
    void setModel(Model& model);


    //--------------------------------------------------------------------------
    // Members
    //--------------------------------------------------------------------------
public:
    Model _model;

    int _n_prescribed_coord;
    int _n_primary_coord;
    int _n_secondary_coord;

    int _n_muscles;
    Array<std::string> _muscle_path;

    int _n_non_muscle_actuators;
    Array<std::string> _non_muscle_actuator_path;

    int _n_actuators;
    int _n_parameters;
    SimTK::Vector _optim_parameters;
    Array<std::string> _optim_parameter_names;

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
    Array<std::string> _secondary_damping_actuator_path;

    SimTK::Vector _optimal_force;
    SimTK::Vector _prev_secondary_value;
    SimTK::Vector _prev_parameters;
    SimTK::Vector _parameter_scale;
    SimTK::Vector _muscle_volumes;
    FunctionSet _cost_muscle_weights;
    std::string _directoryOfSetupFile;
//=============================================================================
};  // END of class COMAK_TOOL

class OSIMPLUGIN_API COMAKSecondaryCoordinate : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(COMAKSecondaryCoordinate, Object)

public:
    OpenSim_DECLARE_PROPERTY(coordinate, std::string, "Path to coordinate in model.")
    OpenSim_DECLARE_PROPERTY(comak_damping, double, "Coefficient to penalize frame-to-frame changes in predicted secondary kinematics.")
    OpenSim_DECLARE_PROPERTY(max_change, double, "Coefficient to penalize frame-to-frame changes in predicted secondary kinematics.")

    COMAKSecondaryCoordinate();
    void constructProperties();
}; //END of class COMAKSecondaryCoordinate

class OSIMPLUGIN_API COMAKSecondaryCoordinateSet : public Set<COMAKSecondaryCoordinate> {
    OpenSim_DECLARE_CONCRETE_OBJECT(COMAKSecondaryCoordinateSet, Set<COMAKSecondaryCoordinateSet>);

public:
    COMAKSecondaryCoordinateSet();
private:
    void constructProperties();
//=============================================================================
};  // COMAKSecondaryCoordinateSet


class OSIMPLUGIN_API COMAKCostFunctionParameter : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(COMAKCostFunctionParameter, Object)

public:
    OpenSim_DECLARE_PROPERTY(actuator, std::string, "Path to actuator in model.")
    OpenSim_DECLARE_PROPERTY(weight, Function, "")
    OpenSim_DECLARE_PROPERTY(desired_activation, Function, "")

    COMAKCostFunctionParameter();
    void constructProperties();
}; //END of class COMAKCostFunctionParameter

class OSIMPLUGIN_API COMAKCostFunctionParameterSet : public Set<COMAKCostFunctionParameter> {
    OpenSim_DECLARE_CONCRETE_OBJECT(COMAKCostFunctionParameterSet, Set<COMAKCostFunctionParameter>)

public:

    COMAKCostFunctionParameterSet();
    void constructProperties();
}; //END of class COMAKCostFunctionParameterSet
}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_COMAK_TOOL_H_


