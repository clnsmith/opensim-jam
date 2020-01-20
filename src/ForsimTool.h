#ifndef OPENSIM_FORSIM_TOOL_H_
#define OPENSIM_FORSIM_TOOL_H_
/* -------------------------------------------------------------------------- *
 *                              ForsimTool.h                                  *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2020 Stanford University and the Authors                *
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


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include "Smith2018ArticularContactForce.h"
#include "H5FileAdapter.h"
#include "OpenSim/Simulation/StatesTrajectory.h"
#include "OpenSim/Simulation/Model/ExternalLoads.h"
#include "OpenSim/Common/FunctionSet.h"


//=============================================================================
//=============================================================================
namespace OpenSim { 

/**
 * A class for recording the kinematics of the generalized coordinates
 * of a model during a simulation.
 *
 * @author Colin Smith
 * @version 1.0
 */
class OSIMPLUGIN_API ForsimTool : public Object {

OpenSim_DECLARE_CONCRETE_OBJECT(ForsimTool, Object);

//=============================================================================
// PROPERTIES
//=============================================================================
public:
	OpenSim_DECLARE_PROPERTY(model_file, std::string,
		"Path to .osim file for analysis.")
	OpenSim_DECLARE_PROPERTY(actuator_input_file, std::string,
		"Storage file (.sto) containing the time varying actuator forces (MuscleName_frc)"  
		"or actuator activations (MuscleName_act) to be applied during the simulation.")
	OpenSim_DECLARE_PROPERTY(external_loads_file,std::string,
		"External loads file (.xml) to apply to the model during the simulation.")
	OpenSim_DECLARE_PROPERTY(prescribed_coordinates_file, std::string, 
		"Storage file (.sto) containing the time varying trajectories for the prescribed coordinates." 
		"Columns labels are 'time' and '/Path/To/Coordinate'")
	OpenSim_DECLARE_PROPERTY(results_directory, std::string,
		"Folder where the results files are output.")
	OpenSim_DECLARE_PROPERTY(results_file_basename, std::string,
		"Prefix to each results file name.")
	OpenSim_DECLARE_PROPERTY(start_time, double,
		"Time to start analysis. Set to -1 to use initial frame in coordinates_file")
	OpenSim_DECLARE_PROPERTY(stop_time, double,
		"Time to stop analysis. Set to -1 to use last frame in coordinates_file")
	OpenSim_DECLARE_PROPERTY(report_time_step, double,
		"The timestep interval to report the results.")
	OpenSim_DECLARE_PROPERTY(minimum_time_step, double,
		"The smallest timestep the integrator is allowed to take.")
	OpenSim_DECLARE_PROPERTY(maximum_time_step, double,
		"The largest timestep the integrator is allowed to take.")
	OpenSim_DECLARE_PROPERTY(integrator_accuracy, double,
		"Accuracy setting for BDF integrator.")
	OpenSim_DECLARE_PROPERTY(constant_muscle_frc, double,
		"Constant muscle force applied for all muscles not listed in actuator_input_file."
		"For each muscle, force=constant_muscle_frc*Max_Isometeric_Force. Set to -1 to ignore")
	OpenSim_DECLARE_LIST_PROPERTY(unconstrained_coordinates, std::string,
		"Paths to Coordinates that will be unconstrained in the simulation." 
		"All Coordinates not listed here or in the prescribed_coordinates_file"
		"will be locked. Note coordinates listed here will override settings"
		"in the .osim file.")
	OpenSim_DECLARE_PROPERTY(use_visualizer,bool,"Use the SimTK visualizer.")
    OpenSim_DECLARE_PROPERTY(verbose,int,"0 - silent, 1 - contact info, 2 - model force info")
	OpenSim_DECLARE_UNNAMED_PROPERTY(AnalysisSet,"Analyses to be performed"
		"during forward simulation.")


//=============================================================================
// METHODS
//=============================================================================
public:
	ForsimTool();
	ForsimTool(std::string settings_file);

	void setModel(Model& aModel);
	void loadModel(const std::string &aToolSetupFileName);
	void run();
	
private:
    void setNull();
    void constructProperties();
	void initializeCoordinates();
	void initializeActuators(SimTK::State& state);
	void applyExternalLoads();
    void initializeStartStopTimes();
    void printDebugInfo(const SimTK::State& state);
	
//=============================================================================
// DATA
//=============================================================================

	
private:
	Model _model;
	ExternalLoads _external_loads;

	std::vector<std::string> _prescribed_frc_actuator_paths;
	std::vector<std::string> _prescribed_act_actuator_paths;
	std::vector<std::string> _prescribed_control_actuator_paths;
	std::vector<std::string> _constant_act_msl_paths;
	FunctionSet _frc_functions;
	FunctionSet _act_functions;

    TimeSeriesTable _actuator_table;
    TimeSeriesTable _coord_table;

    std::string _directoryOfSetupFile;
//=============================================================================
};  // END of class ForsimTool

}; //namespace



#endif // #ifndef OPENSIM_FORSIM_TOOL_H_
