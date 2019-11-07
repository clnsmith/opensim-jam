/* -------------------------------------------------------------------------- *
 *                                ForsimTool.cpp                              *
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


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Simulation/Model/Model.h>
#include "ForsimTool.h"
#include "HelperFunctions.h"
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/OpenSim.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/FunctionSet.h>
#include <OpenSim\Actuators\Millard2012EquilibriumMuscle.h>
using namespace OpenSim;


//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================

ForsimTool::ForsimTool() : Object()
{
	setNull();
	constructProperties();

}

ForsimTool::ForsimTool(std::string settings_file) : Object(settings_file) {
	constructProperties();
	updateFromXMLDocument();
	loadModel(settings_file);
	
    _directoryOfSetupFile = IO::getParentDirectory(settings_file);	
}


//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void ForsimTool::setNull()
{
    setAuthors("Colin Smith");

}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ForsimTool::constructProperties()
{
	constructProperty_model_file("");
	constructProperty_actuator_input_file("");
	constructProperty_external_loads_file("");
	constructProperty_prescribed_coordinates_file("");	
	constructProperty_results_directory(".");
	constructProperty_results_file_basename("");
	constructProperty_start_time(-1);
	constructProperty_stop_time(-1);
	constructProperty_integrator_accuracy(0.000001);
	constructProperty_report_time_step(0.01);
	constructProperty_minimum_time_step(0.00000001);
	constructProperty_maximum_time_step(0.01);
	constructProperty_constant_muscle_frc(-1);
	constructProperty_unconstrained_coordinates();
	constructProperty_use_visualizer(false);
	constructProperty_AnalysisSet(AnalysisSet());
}


void ForsimTool::setModel(Model& aModel)
{
	_model = aModel;
	set_model_file(_model.getDocumentFileName());
}

void ForsimTool::run() 
{	
	
	SimTK::State state = _model.initSystem();
	
	if (get_use_visualizer()) {
		_model.setUseVisualizer(true);
	}	
	
	//Add Analysis set
	AnalysisSet aSet = get_AnalysisSet();
	int size = aSet.getSize();

    for(int i=0;i<size;i++) {
        Analysis *analysis = aSet.get(i).clone();
        _model.addAnalysis(analysis);
    }
	
	//Apply External Loads
	applyExternalLoads();
	
	//Prescribe Coordinates in the Model
	initializeCoordinates();
	state = _model.initSystem();
	
	//Apply Muscle Forces
	initializeActuators(state);
	state = _model.initSystem();
	
	//Set Start and Stop Times
    initializeStartStopTimes();

	//Allocate Results Storage
	TimeSeriesTable q_table;
	TimeSeriesTable u_table;

	SimTK::RowVector q_row(_model.getNumCoordinates());
	SimTK::RowVector u_row(_model.getNumCoordinates());

	std::vector<std::string> q_names;
	std::vector<std::string> u_names;

	for (auto& coord : _model.updComponentList<Coordinate>()) {
					
		q_names.push_back(coord.getAbsolutePathString() + "/value");
		u_names.push_back(coord.getAbsolutePathString() + "/speed");
	}

	q_table.setColumnLabels(q_names);
	u_table.setColumnLabels(u_names);

	_model.equilibrateMuscles(state);

	//Setup Visualizer
	if (get_use_visualizer()) {
		_model.updMatterSubsystem().setShowDefaultGeometry(false);
		SimTK::Visualizer& viz = _model.updVisualizer().updSimbodyVisualizer();
		viz.setBackgroundColor(SimTK::White);
		viz.setShowSimTime(true);		
	}

	//Set Integrator
	SimTK::CPodesIntegrator integrator(_model.getSystem(), SimTK::CPodes::BDF, SimTK::CPodes::Newton);
	integrator.setAccuracy(get_integrator_accuracy());
	integrator.setMinimumStepSize(get_minimum_time_step());
	integrator.setMaximumStepSize(get_maximum_time_step());

	SimTK::TimeStepper timestepper(_model.getSystem(), integrator);
	state.setTime(get_start_time());
	timestepper.initialize(state);

	//Integrate Forward in Time
	double dt = get_report_time_step();
	int nSteps = round((get_stop_time() - get_start_time()) / dt);
	AnalysisSet& analysisSet = _model.updAnalysisSet();

	std::cout << std::endl;
	std::cout << std::endl;
	std::cout << "Performing Forward Dynamic Simulation" << std::endl;
	std::cout << "Start Time: " << get_start_time() << std::endl;
	std::cout << "Stop Time: " << get_stop_time() << std::endl;
	std::cout << std::endl;
	
	//SimTK::State& s= timestepper.updIntegrator().updAdvancedState();

	for (int i = 0; i <= nSteps; ++i) {

		double t = get_start_time() + i * dt;
		std::cout << "Time:" << t << std::endl;

		//Set Prescribed Muscle Forces
		
		for (int j = 0; j < _prescribed_frc_actuator_paths.size();++j) {
           
			std::string actuator_path = _prescribed_frc_actuator_paths[j];
			ScalarActuator& actuator = _model.updComponent<ScalarActuator>(actuator_path);
			double value = _frc_functions.get(actuator_path +"_frc").calcValue(SimTK::Vector(1,t));
			actuator.setOverrideActuation(state, value);
		}
		


		timestepper.stepTo(t);
        state = timestepper.updIntegrator().getState();

        for (int j = 0; j < _prescribed_frc_actuator_paths.size();++j) {
			std::string actuator_path = _prescribed_frc_actuator_paths[j];
			ScalarActuator& actuator = _model.updComponent<ScalarActuator>(actuator_path);
		}

        for (int j = 0; j < _prescribed_act_actuator_paths.size(); ++j) {
            std::string actuator_path = _prescribed_frc_actuator_paths[j];
			Muscle& actuator = _model.updComponent<Muscle>(actuator_path);
        }

		//Record parameters
		if (i == 0) {
			analysisSet.begin(state);
		}
		else {
			analysisSet.step(state, i);
		}

		
		int j = 0;
		for (const auto& coord : _model.updComponentList<Coordinate>()) {
			if (coord.getMotionType() == Coordinate::MotionType::Rotational) {
				q_row(j) = coord.getValue(state)*180/SimTK::Pi;
				u_row(j) = coord.getSpeedValue(state)*180/SimTK::Pi;
			}
			else {
				q_row(j) = coord.getValue(state);
				u_row(j) = coord.getSpeedValue(state);
			}
			j++;
		}
		q_table.appendRow(state.getTime(), q_row);
		u_table.appendRow(state.getTime(), u_row);
	}


	//Print Results
	STOFileAdapter sto;
	std::string basefile = get_results_directory() + "/" + get_results_file_basename();
	
	q_table.addTableMetaData("header", std::string("CoordinateValues"));
	q_table.addTableMetaData("nRows", std::to_string(q_table.getNumRows()));
	q_table.addTableMetaData("nColumns", std::to_string(q_table.getNumColumns()+1));
	q_table.addTableMetaData("inDegrees", std::string("yes"));
	sto.write(q_table, basefile + "_values.sto");
	
	u_table.addTableMetaData("header", std::string("CoordinateSpeeds"));
	u_table.addTableMetaData("nRows", std::to_string(u_table.getNumRows()));
	u_table.addTableMetaData("nColumns", std::to_string(u_table.getNumColumns()+1));
	u_table.addTableMetaData("inDegrees", std::string("yes"));
	sto.write(u_table, basefile + "_speeds.sto");
	
	_model.updAnalysisSet().printResults(get_results_file_basename(), get_results_directory());

	std::cout << "\nSimulation complete." << std::endl;
	std::cout << "Printed results to: " + get_results_directory() << std::endl;
}

void ForsimTool::initializeStartStopTimes() {
    if (get_start_time() != -1 && get_stop_time() != -1) {
		return;
	}

    double start = 0;
    double end = 0;
    double act_start = -2;
    double act_end = -2;
    double coord_start = -2;
    double coord_end = -2;

    if (get_actuator_input_file() != "") {
        std::vector<double> act_time = _actuator_table.getIndependentColumn();
        act_start = act_time[0];
        act_end = act_time.back();
    }
    if (get_prescribed_coordinates_file() != "") {
        std::vector<double> coord_time = _coord_table.getIndependentColumn();
        coord_start = coord_time[0];
        coord_end = coord_time.back();
    }

    if (act_start == -2 && coord_start == -2) {
        OPENSIM_THROW(Exception, "No actuator_input_file or "
        "prescribed_coordinates_files defined." 
        "You must set start_time and stop_time to non-negative values.")
    }

    if (act_start != -2 && coord_start != -2) {
        if (act_start != coord_start || act_end != coord_end) {
            OPENSIM_THROW(Exception, "The start and stop times of the "
                "actuator_input_file and prescribed coordinate files do not match."
                "You must set start_time and stop_time to non-negative values.")
        }
    }

    if (get_start_time() == -1) {
        if (coord_start != -2)
            set_start_time(coord_start);
        else
            set_start_time(act_start);;
	}
	if (get_stop_time() == -1) {
		if (coord_end != -2)
            set_stop_time(coord_end);
        else
            set_stop_time(act_end);;
	}
}

void ForsimTool::initializeActuators(SimTK::State& state) {
	if (!(get_actuator_input_file() == "")){
		STOFileAdapter actuator_file;
		TimeSeriesTable _actuator_table = actuator_file.read(get_actuator_input_file());

		int nDataPt = _actuator_table.getNumRows();
		std::vector<std::string> labels = _actuator_table.getColumnLabels();
		std::vector<double> time = _actuator_table.getIndependentColumn();

		//Set minimum activation
		PrescribedController* control = new PrescribedController();

		for (int i = 0; i < labels.size(); ++i) {
			std::vector<std::string> split_label = split_string(labels[i], "_");

			if (split_label.back() == "frc") {
				std::string actuator_path = erase_sub_string(labels[i], "_frc");

				try {
					ScalarActuator& actuator = _model.updComponent<ScalarActuator>(actuator_path);

					actuator.overrideActuation(state, true);

					_prescribed_frc_actuator_paths.push_back(actuator_path);

					SimTK::Vector values = _actuator_table.getDependentColumn(labels[i]);
					SimmSpline* frc_function = new SimmSpline(nDataPt, &time[0], &values[0], actuator_path + "_frc");

					_frc_functions.adoptAndAppend(frc_function);
				}
				catch (ComponentNotFoundOnSpecifiedPath) {
					OPENSIM_THROW(Exception,
						"Actuator: " + actuator_path + " not found in model. "
						"Did you use absolute path?")
				}
			}

			if (split_label.back() == "act") {
				std::string actuator_path = erase_sub_string(labels[i], "_act");

				try {
					Millard2012EquilibriumMuscle& msl = _model.updComponent<Millard2012EquilibriumMuscle>(actuator_path);

					_prescribed_act_actuator_paths.push_back(actuator_path);

					SimTK::Vector values = _actuator_table.getDependentColumn(labels[i]);
					SimmSpline* act_function = new SimmSpline(nDataPt, &time[0], &values[0], actuator_path + "_act");

					control->addActuator(msl);

					control->prescribeControlForActuator(msl.getName(), act_function);

					//_act_functions.adoptAndAppend(act_function);
					msl.set_ignore_activation_dynamics(true);
				}
				catch (ComponentNotFoundOnSpecifiedPath) {
					OPENSIM_THROW(Exception,
						"Muscle: " + actuator_path + " not found in model. "
						"Did you use absolute path? Is it a Millard2012EquilibriumMuscle?")
				}
			}

			if (split_label.back() == "control") {
				std::string actuator_path = erase_sub_string(labels[i], "_control");

				try {
					ScalarActuator& actuator = _model.updComponent<ScalarActuator>(actuator_path);
					_prescribed_control_actuator_paths.push_back(actuator_path);
					SimTK::Vector values = _actuator_table.getDependentColumn(labels[i]);

					SimmSpline* control_function = new SimmSpline(nDataPt, &time[0], &values[0], actuator_path + "_control");

					control->addActuator(actuator);

					control->prescribeControlForActuator(actuator.getName(), control_function);

					std::cout << "Control Prescribed: " << actuator_path << std::endl;
				}
				catch (ComponentNotFoundOnSpecifiedPath) {
					OPENSIM_THROW(Exception,
						"Actuator: " + actuator_path + " not found in model. "
						"Did you use absolute path?")
				}

			}
		}

		//Output to Screen
		if (_prescribed_frc_actuator_paths.size() > 0) {
			std::cout << std::endl;
			std::cout << "Force Prescribed:" << std::endl;
			for (std::string& name : _prescribed_frc_actuator_paths) {
				std::cout << name << std::endl;
			}
			std::cout << std::endl;
		}

		if (_prescribed_act_actuator_paths.size() > 0) {
			std::cout << "Activation Prescribed:" << std::endl;
			for (std::string& name : _prescribed_act_actuator_paths) {
				std::cout << name << std::endl;
			}
			std::cout << std::endl;
		}

		if (_prescribed_control_actuator_paths.size() > 0) {
			std::cout << "Control Prescribed:" << std::endl;
			for (std::string& name : _prescribed_control_actuator_paths) {
				std::cout << name << std::endl;
			}
			std::cout << std::endl;
		}
	}

	//Set Constant Muscle Activation
	if (get_constant_muscle_frc() > -1) {
		std::cout << "Constant Muscle Force Multiplier: " << get_constant_muscle_frc() << std::endl;
		for (Muscle& msl : _model.updComponentList<Muscle>()) {
			std::string msl_path = msl.getAbsolutePathString();

			if (contains_string(_prescribed_frc_actuator_paths, msl_path)) {
				continue;
			}
			if (contains_string(_prescribed_act_actuator_paths, msl_path)) {
				continue;
			}
			if (contains_string(_prescribed_control_actuator_paths, msl_path)) {
				continue;
			}
			
			msl.overrideActuation(state, true);

			if (msl.getConcreteClassName() == "Millard2012EquilibriumMuscle") {
				msl.setIgnoreTendonCompliance(state, true);
				msl.setIgnoreActivationDynamics(state, true);
			}

			_prescribed_frc_actuator_paths.push_back(msl_path);
			
			Constant* frc_function = new Constant(0.0);
			frc_function->setName(msl_path + "_frc");
				//SimmSpline(nDataPt, &time[0], &values[0], msl_path + "_frc");

			_frc_functions.adoptAndAppend(frc_function);
			std::cout << msl_path << std::endl;
		}
		std::cout << std::endl;
	}
}



void ForsimTool::initializeCoordinates() {
	
	for (Coordinate& coord : _model.updComponentList<Coordinate>()) {
		coord.set_locked(true);
	}

	std::cout << "\nUnconstrained Coordinates:" << std::endl;
	for (int i = 0; i < getProperty_unconstrained_coordinates().size(); ++i) {
		std::string coord_path = get_unconstrained_coordinates(i);

		try {
			Coordinate& coord = _model.updComponent<Coordinate>(coord_path);
			coord.set_locked(false);
			std::cout <<  coord_path << std::endl;
		}
		catch(ComponentNotFoundOnSpecifiedPath){OPENSIM_THROW(Exception,
			"Unconstrained Coordinate: " +  coord_path + "Not found in model."
			"Did you use absolute path?") }
	}
	
    
	//Load prescribed coordinates file
	STOFileAdapter coord_file;
    if (get_prescribed_coordinates_file() != "") {

        std::string saveWorkingDirectory = IO::getCwd();
        IO::chDir(_directoryOfSetupFile);

        try {
             _coord_table = coord_file.read(get_prescribed_coordinates_file());
      
        } catch(...) { // Properly restore current directory if an exception is thrown
            IO::chDir(saveWorkingDirectory);
            throw;
        }
        IO::chDir(saveWorkingDirectory);
        
        std::vector<std::string> labels = _coord_table.getColumnLabels();

        int nDataPt = _coord_table.getNumRows();
        std::vector<double> time = _coord_table.getIndependentColumn();

        std::cout << "\nPrescribed Coordinates:" << std::endl;
        for (int i = 0; i < labels.size(); ++i) {
            try {
                Coordinate& coord = _model.updComponent<Coordinate>(labels[i]);
                SimTK::Vector values = _coord_table.getDependentColumn(labels[i]);
                if (coord.getMotionType() == Coordinate::MotionType::Rotational) {
                    values *= SimTK::Pi / 180;
                }

                SimmSpline function = SimmSpline(nDataPt, &time[0], &values[0], coord.getName() + "_prescribed");
                coord.set_prescribed(true);
                coord.set_prescribed_function(function);
                coord.set_locked(false);

                std::cout << labels[i] << std::endl;
            }
            catch (ComponentNotFoundOnSpecifiedPath) { OPENSIM_THROW(Exception, "Prescribed Coordinate: " + labels[i] + "was not found in model. Did you use absolute path?") }
        }
        std::cout << std::endl;
    }
}

void ForsimTool::applyExternalLoads()
{
	const std::string& aExternalLoadsFileName = get_external_loads_file();

	if (aExternalLoadsFileName == "" || aExternalLoadsFileName == "Unassigned") {
		std::cout << "No external loads will be applied (external loads file not specified)." << std::endl;
		return;
	}

	// This is required so that the references to other files inside ExternalLoads file are interpreted 
    // as relative paths
    std::string savedCwd = IO::getCwd();
    IO::chDir(IO::getParentDirectory(aExternalLoadsFileName));
    // Create external forces
    ExternalLoads* externalLoads = nullptr;
    try {
        externalLoads = new ExternalLoads(aExternalLoadsFileName, true);
        _model.addModelComponent(externalLoads);
    }
    catch (const Exception &ex) {
        // Important to catch exceptions here so we can restore current working directory...
        // And then we can re-throw the exception
        std::cout << "Error: failed to construct ExternalLoads from file " << aExternalLoadsFileName;
        std::cout << ". Please make sure the file exists and that it contains an ExternalLoads";
        std::cout << "object or create a fresh one." << std::endl;
        if (getDocument()) IO::chDir(savedCwd);
        throw(ex);
    }

    // copy over created external loads to the external loads owned by the tool
    _external_loads = *externalLoads;

    IO::chDir(savedCwd);
    return;
}
/*
void ForsimTool::formQandUMatrixFromFile() {
	
	Storage store(get_coordinates_file());
			std::cout << _model->getName() << std::endl;
	const CoordinateSet& coordinateSet = _model->getCoordinateSet();

	std::cout << coordinateSet.get(0).getName() << std::endl;
	
	if (store.isInDegrees()) {
		_model->getSimbodyEngine().convertDegreesToRadians(store);
	}

	if (get_resample_step_size() != -1) {
		store.resampleLinear(get_resample_step_size());
	}

	if (get_lowpass_filter_frequency() != -1) {
		store.pad(store.getSize() / 2);
		store.lowpassIIR(get_lowpass_filter_frequency());
	}
	
	
	//Set Start and Stop Times
	store.getTimeColumn(_time);
	 

	if (get_start_time() == -1) {
		set_start_time(_time.get(0));
	}
	if (get_stop_time() == -1) {
		set_stop_time(_time.getLast());
	}

	//Set number of Frames
	_n_frames = _time.size();
	_n_out_frames = 0;
	for (int i = 0; i < _n_frames; ++i) {
		if (_time[i] < get_start_time()) { continue; }
		if (_time[i] > get_stop_time()) { break; };
		_n_out_frames++;
	}
	//Gather Q and U values
	Array<std::string> col_labels = store.getColumnLabels();
	
	Array<int> q_col_map(-1,_model->getNumCoordinates());
	//q_col_map = -1;
	
	for (int i = 0; i < col_labels.size(); ++i) {
		std::vector<std::string> split_label = split_string(col_labels[i], "/");

		int j = 0;
		for (const Coordinate& coord : _model->getComponentList<Coordinate>()) {
			if (contains_string(split_label, coord.getName())) {
				if (split_label.back() == "value" || split_label.back() == coord.getName()){
					q_col_map[j] = i;
				}
			}
			j++;
		}

	}
	
	_q_matrix.resize(_n_frames, _model->getNumCoordinates());
	_u_matrix.resize(_n_frames, _model->getNumCoordinates());
	
	_q_matrix = 0;
	_u_matrix = 0;

	int j = 0;
	for (const Coordinate& coord : _model->getComponentList<Coordinate>()) {

		if (q_col_map[j] != -1) {
			double* data = NULL;
			store.getDataColumn(col_labels[q_col_map[j]], data);
			for (int i = 0; i < _n_frames; ++i) {
				_q_matrix(i,j) = data[i];
			}
		}
		else{
			std::cout << "Coordinate Value: " << coord.getName() << " not found in coordinates_file, assuming 0." << std::endl;			
		}
		
		GCVSpline q_spline;
		q_spline.setDegree(5);

		for (int i = 0; i < _n_frames; ++i) {
			Array<double> data;
			store.getDataColumn(col_labels[q_col_map[j]], data);
			q_spline.addPoint(_time[i], data[i]);
		}

		for (int i = 0; i < _n_frames; ++i) {
			SimTK::Vector x(1);
			x(0) = _time[i];

			std::vector<int> u_order = { 0 };
			_u_matrix(i, j) = q_spline.calcDerivative(u_order, x);
		}		
		j++;
	}
}*/                                                                              

void ForsimTool::loadModel(const std::string &aToolSetupFileName)
{
    
	OPENSIM_THROW_IF(get_model_file().empty(), Exception,
            "No model file was specified (<model_file> element is empty) in "
            "the Setup file. ");
    std::string saveWorkingDirectory = IO::getCwd();
    std::string directoryOfSetupFile = IO::getParentDirectory(aToolSetupFileName);
    IO::chDir(directoryOfSetupFile);

    std::cout<<"ForsimTool "<< getName() <<" loading model '"<<get_model_file() <<"'"<< std::endl;

	Model model;

    try {
        model = Model(get_model_file());
		model.finalizeFromProperties();
        
    } catch(...) { // Properly restore current directory if an exception is thrown
        IO::chDir(saveWorkingDirectory);
        throw;
    }
    _model = model;
    IO::chDir(saveWorkingDirectory);
}