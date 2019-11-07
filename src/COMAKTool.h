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
#include "osimPluginDLL.h"

namespace OpenSim { 

//=============================================================================
//=============================================================================
/**
 * A class to store and generate the settings parameters for COMAK.
 *
 * @author Colin Smith
 * @version 1.0
 */
class OSIMPLUGIN_API COMAKTool : public Component{
	OpenSim_DECLARE_CONCRETE_OBJECT(COMAKTool, Component)

public:
	OpenSim_DECLARE_PROPERTY(ik_motion_file, std::string, "Location of the inverse kinematics input .mot file.")
		OpenSim_DECLARE_PROPERTY(results_dir, std::string, "Path to directory to write output results.")
		OpenSim_DECLARE_PROPERTY(model_file, std::string, "Location of model to use in COMAK simulation.")
		OpenSim_DECLARE_PROPERTY(comak_settings_file, std::string, "Path to .xml file to save comak settings")
		OpenSim_DECLARE_PROPERTY(external_loads_file, std::string, "External loads .xml file.")
		OpenSim_DECLARE_PROPERTY(results_prefix, std::string, "Prefix to all results files names.")
		OpenSim_DECLARE_PROPERTY(use_visualizer, bool, "Use SimTK visualizer to display simulations in progress.")
		OpenSim_DECLARE_PROPERTY(print_input_kinematics, bool, "Print processed input q's, u's, and udots.")

		OpenSim_DECLARE_PROPERTY(perform_secondary_constraint_sim, bool,
			"Perform forward simulation where secondary_coupled_coord is prescribed and secondardy coordinates are unconstrained to find constraint functions for IK.")
		OpenSim_DECLARE_OPTIONAL_PROPERTY(secondary_coupled_coord, std::string, "Name of coordinate to prescribe in simulation.")
		//OpenSim_DECLARE_OPTIONAL_PROPERTY(secondary_coupled_coord_trajectory, Function, "Coordinate values vs time to prescribe during simulation, must start at time=0.0.")
		OpenSim_DECLARE_PROPERTY(secondary_constraint_sim_settle_time, double, "Initial settle period where secondary_coupled_coord is locked and secondary coords settle.")
		OpenSim_DECLARE_PROPERTY(secondary_coupled_coord_start_value, double, "Initial value for secondary_coupled_coord. In degrees for rotational coordinates.")
		OpenSim_DECLARE_PROPERTY(secondary_coupled_coord_stop_value, double, "Final value for secondary_coupled_coord. In degrees for rotational coordinates.")
		OpenSim_DECLARE_PROPERTY(secondary_constraint_sim_time, double, "Length of time to linearly prescribe secondary_couple_coord through the start and end value.")
		OpenSim_DECLARE_PROPERTY(secondary_constraint_sim_integrator_accuracy, double, "Integrator tolerance for forward simulation.")
		OpenSim_DECLARE_PROPERTY(print_secondary_constraint_sim_results, bool, "Print results .sto file for secondary_constraint_sim")
		OpenSim_DECLARE_PROPERTY(secondary_constraint_funcs, FunctionSet, "Functions constraining the secondary coordinates to the secondary_coupled_coord.")

		OpenSim_DECLARE_PROPERTY(perform_inverse_kinematics, bool, "Perform Inverse Kinematics")
		OpenSim_DECLARE_OPTIONAL_PROPERTY(trc_kinematics_file, std::string, "Path to .trc file with marker kinematics")
		OpenSim_DECLARE_OPTIONAL_PROPERTY(ik_settings_file, std::string, "Path to Inverse Kinematics settings .xml file.")
		OpenSim_DECLARE_OPTIONAL_PROPERTY(print_ik_model, bool, "Print model .osim file with kinematic coupler constraints.")
		OpenSim_DECLARE_OPTIONAL_PROPERTY(ik_print_model_file, std::string, "Path to output .osim file.")

		OpenSim_DECLARE_PROPERTY(verbose, int, "Level of debug information reported (0: low, 1: medium, 2: high)")
		OpenSim_DECLARE_PROPERTY(start_time, double, "First time step of COMAK simulation.")
		OpenSim_DECLARE_PROPERTY(stop_time, double, "Last time step of COMAK simulation.")
		OpenSim_DECLARE_PROPERTY(time_step, double, "Time increment between steps in COMAK simulation.Set to -1 to use input kinematics time step.")
		OpenSim_DECLARE_PROPERTY(lowpass_filter_frequency, double, "Lowpass filter frequency for input kinematics. Set to -1 to use raw data")

		OpenSim_DECLARE_LIST_PROPERTY(prescribed_coordinates, std::string, "List the Prescribed Coordinates in the model.")
		OpenSim_DECLARE_LIST_PROPERTY(primary_coordinates, std::string, "List the Primary Coordinates in the model.")
		OpenSim_DECLARE_LIST_PROPERTY(secondary_coordinates, std::string, "List the Secondary Coordinates in the model.")
		OpenSim_DECLARE_LIST_PROPERTY(secondary_comak_damping, double, "List of damping values for each secondary coordinate to penalize frame to frame changes.")

		OpenSim_DECLARE_PROPERTY(max_iterations, int, "Max number of iterations per time step for the COMAK optimization.")
		OpenSim_DECLARE_PROPERTY(max_change_rotation, double, "Maximum change in rotational secondary coordinates between COMAK iterations")
		OpenSim_DECLARE_PROPERTY(max_change_translation, double, "Maximum change in translational secondary coordinates between COMAK iterations")
		OpenSim_DECLARE_PROPERTY(udot_tolerance, double, "Tolerance on achieving the desired udots.")
		OpenSim_DECLARE_PROPERTY(udot_worse_case_tolerance, double, "Max error in udot acceptable if no iterations converge, if max udot error > worse case tolerance the previous time step is used.")
		OpenSim_DECLARE_PROPERTY(unit_udot_epsilon, double, "Perturbation applied to secondary_coords when computing the linearized version of the COMAK acceleration constraints.")
		OpenSim_DECLARE_PROPERTY(comak_damping_multiplier, double, "Multiplier applied to Spring Generalized Force viscosity in the model when computing COMAK damping");
        OpenSim_DECLARE_PROPERTY(contact_energy_weight, double, "Weighting term for contact energy in COMAK cost function")

	    OpenSim_DECLARE_PROPERTY(primary_reserve_actuator_strength,double,"Set to -1 to ignore. Otherwise set value for optimal strength of idealized torque actuators added to primary coordinates. ")
	    OpenSim_DECLARE_PROPERTY(secondary_reserve_actuator_strength, double, "Set to -1 to ignore. Otherwise set value for optimal strength of idealized torque actuators added to primary coordinates. ")

	    OpenSim_DECLARE_PROPERTY(equilibriate_secondary_coordinates_at_start, bool, "Perform a forward simulation to settle secondary coordinates into equilbrium at initial time step of COMAK.")
	    OpenSim_DECLARE_PROPERTY(settle_accuracy, double, "Integrator accuracy for initializing forward simulation.")
	    OpenSim_DECLARE_PROPERTY(settle_threshold, double, "Maximum change in secondary coordinates between timesteps that defines equilibrium.")
	    OpenSim_DECLARE_PROPERTY(print_settle_sim_results, bool, "Print the coordinate and speed values during the forward simulation to settle_sim_results_dir.")
	    OpenSim_DECLARE_PROPERTY(settle_sim_results_prefix, std::string, "Prefix to settle simulation results file names.")
	    OpenSim_DECLARE_PROPERTY(settle_sim_results_dir, std::string, "Path to results directory to print forward simulation results.")
	    OpenSim_DECLARE_PROPERTY(settle_sim_viscosity_multiplier,double, 
		    "Multiplier applied to SpringGeneralizedForce viscosity in the model during settle simulation. Set to -1 to ignore.")
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

public:
	void initialize();
	void run();
	void performIKSecondaryConstraintSimulation();
	void performIK();
	SimTK::Vector equilibriateSecondaryCoordinates();
	void performCOMAK();
	void setModel(Model& model);
	void extractKinematicsFromFile();
	void applyExternalLoads();
	void printCOMAKascii();
	void setStateFromComakParameters(SimTK::State& state, const SimTK::Vector parameters);
    SimTK::Vector computeMuscleVolumes();
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
//=============================================================================
};  // END of class COMAK_TOOL

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_COMAK_TOOL_H_


