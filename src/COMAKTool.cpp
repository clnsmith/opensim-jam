/* -------------------------------------------------------------------------- *
 *                                 COMAKTool.cpp                              *
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
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim.h>
#include "COMAKTool.h"
#include "COMAKTarget.h"
#include "HelperFunctions.h"
#include "Smith2018ArticularContactForce.h"
#include "Blankevoort1991Ligament.h"

using namespace OpenSim;
using namespace SimTK;

/*
class ForceArrowGenerator : public DecorationGenerator {
public:
	ForceArrowGenerator(const Model& model, const ExternalForce& extFrc)
		: _model(model), _extFrc(extFrc) {}

	virtual void generateDecorations(const State& state, Array_<DecorativeGeometry>& geometry) override {
		const Vec3 frcColors[] = { Red,Orange,Cyan };
		const Vec3 momColors[] = { Blue,Green,Purple };
		_model.realizeVelocity(state);

		const Vec3& frc = _extFrc.getForceAtTime(state.getTime());
		const Vec3& pnt = _extFrc.getPointAtTime(state.getTime());

		static const Real ForceScale = .25;
		Real frcScale = ForceScale;
		Real  frcMag = frc.norm();
		int frcThickness = 1;
		while (frcMag > 10) {
			frcThickness++, frcScale /= 10, frcMag /= 10;
		}
		DecorativeLine frcLine(pnt,
			pnt + frcScale * frc);
		frcLine.setLineThickness(2 * frcThickness);
		frcLine.setColor(Red);
		geometry.push_back(frcLine);

	}
private:
	const Model& _model;
	const ExternalForce& _extFrc;
};*/
//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================

//_____________________________________________________________________________
/**
 * Default constructor.
 */
COMAKTool::COMAKTool() 
{
	constructProperties();
}

COMAKTool::COMAKTool(const std::string file) : Component(file) {
	constructProperties();
	updateFromXMLDocument();
	finalizeFromProperties();
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void COMAKTool::constructProperties()
{
	constructProperty_model_file("");
	constructProperty_results_dir("");
	constructProperty_comak_settings_file("");
	constructProperty_external_loads_file("");
	constructProperty_results_prefix("");
	constructProperty_use_visualizer(false);
	constructProperty_print_input_kinematics(false);
	
	constructProperty_perform_secondary_constraint_sim(false);
	constructProperty_secondary_coupled_coord("");
	constructProperty_secondary_constraint_sim_settle_time(1.0);
	constructProperty_secondary_constraint_sim_time(1.0);
	constructProperty_secondary_coupled_coord_start_value(0.0);
	constructProperty_secondary_coupled_coord_stop_value(0.0);
	constructProperty_secondary_constraint_sim_integrator_accuracy(0.0001);
	constructProperty_print_secondary_constraint_sim_results(false);
	constructProperty_secondary_constraint_funcs(FunctionSet());

	constructProperty_perform_inverse_kinematics(false);
	constructProperty_ik_settings_file("");
	constructProperty_trc_kinematics_file("");
	constructProperty_ik_motion_file("");
	constructProperty_print_ik_model(false);
	constructProperty_ik_print_model_file("");

	constructProperty_verbose(0);
	constructProperty_start_time(-1);
	constructProperty_stop_time(-1);
	constructProperty_time_step(-1);
	constructProperty_lowpass_filter_frequency(-1);

	constructProperty_prescribed_coordinates();
	constructProperty_primary_coordinates();
	constructProperty_secondary_coordinates();
	constructProperty_secondary_comak_damping();

	constructProperty_max_iterations(50);
	constructProperty_max_change_rotation(0.05);
	constructProperty_max_change_translation(0.005);
	constructProperty_udot_tolerance(1.0);
	constructProperty_udot_worse_case_tolerance(50.0);
	constructProperty_unit_udot_epsilon(0.000005);
	constructProperty_comak_damping_multiplier(1.0);
    constructProperty_contact_energy_weight(0.0);

	constructProperty_primary_reserve_actuator_strength(-1);
	constructProperty_secondary_reserve_actuator_strength(-1);
	constructProperty_equilibriate_secondary_coordinates_at_start(true);
	constructProperty_settle_threshold(0.00001);
	constructProperty_settle_accuracy(0.00001);
	constructProperty_settle_sim_results_prefix("");
	constructProperty_settle_sim_results_dir("");
	constructProperty_print_settle_sim_results(false);
	constructProperty_settle_sim_viscosity_multiplier(1.0);
}

void COMAKTool::setModel(Model& model){
	_model = model;
}

/** ===================================================================
* INITIALIZE
* =====================================================================
*
*/
void COMAKTool::initialize()
{
	setModel(Model(get_model_file()));

	if (get_use_visualizer()) {
		_model.setUseVisualizer(true);
	}
	_model.initSystem();
	//if (get_verbose() > 3) {
		for (auto& ef : _model.updComponentList<Smith2018ArticularContactForce>()) {
			//ef.set_verbose(0);
		}
	//}



	//Verfiy Coordinate Properties
	
	for (Coordinate& coord : _model.updComponentList<Coordinate>()) {
		std::string name = coord.getName();
		std::string path = coord.getAbsolutePathString();

		//Reset to full path
		if (get_secondary_coupled_coord() == name) {
			set_secondary_coupled_coord(path);
		}

		int ind = getProperty_prescribed_coordinates().findIndex(name);
		if (ind > -1) {
			set_prescribed_coordinates(ind, path);
		}
		
		ind = getProperty_primary_coordinates().findIndex(name);
		if (ind > -1) {
			set_primary_coordinates(ind, path);
		}

		ind = getProperty_secondary_coordinates().findIndex(name);
		if (ind > -1) {
			set_secondary_coordinates(ind, path);
		}

		bool isPrescribed = (getProperty_prescribed_coordinates().findIndex(path) > -1);
		bool isPrimary = (getProperty_primary_coordinates().findIndex(path) > -1);
		bool isSecondary = (getProperty_secondary_coordinates().findIndex(path) > -1);
		
		if (!isPrescribed && !isPrimary && !isSecondary) {
			append_prescribed_coordinates(path);

			std::cout << "WARNING: Coordinate (" << name <<
				") was not listed in COMAKTool params file. Assumed PRESCRIBED."
				<< std::endl;
		}

		int counted = 0;
		if (isPrescribed)
			counted++;
		if (isPrimary)
			counted++;
		if (isSecondary)
			counted++;
	
		if (counted > 1) {
			OPENSIM_THROW(Exception, "Coordinate: " + name + " was listed as multiple COMAKTool coordinate types (Prescribed, Primary, Secondary).")
		}
	}

	//Make sure Coordinate exists in model and no duplicates	
	std::string name = get_secondary_coupled_coord();
	try { _model.getComponent<Coordinate>(name); }
	catch (Exception) {
		OPENSIM_THROW(Exception, "secondary_coupled_coord: " + name + " not found in model.")
	}
	
	for (int i = 0; i < getProperty_prescribed_coordinates().size(); ++i) {
		std::string name = get_prescribed_coordinates(i);
		try { _model.getComponent<Coordinate>(name);}
		catch (Exception) {
			OPENSIM_THROW(Exception, "prescribed_coordinate: " + name + " not found in model.")
		}

		int n = 0;
		for (int j = 0; j < getProperty_prescribed_coordinates().size(); ++j) {
			if (name == get_prescribed_coordinates(j)) n++;
		}
		OPENSIM_THROW_IF(n>1, Exception, name + "listed multiple times in prescribed_coordinates")
	}

	for (int i = 0; i < getProperty_primary_coordinates().size(); ++i) {
		std::string name = get_primary_coordinates(i);
		try { _model.getComponent<Coordinate>(name); }
		catch (Exception) {
			OPENSIM_THROW(Exception, "primary_coordinate: " + name + "not found in model.")
		}

		int n = 0;
		for (int j = 0; j < getProperty_primary_coordinates().size(); ++j) {
			if (name == get_primary_coordinates(j)) n++;
		}
		OPENSIM_THROW_IF(n>1, Exception, name + "listed multiple times in primary_coordinates")
	}

	for (int i = 0; i < getProperty_secondary_coordinates().size(); ++i) {
		std::string name = get_secondary_coordinates(i);
		try { _model.getComponent<Coordinate>(name); }
		catch (Exception){
			OPENSIM_THROW(Exception,"secondary_coordinate: " + name + "not found in model.")
		}

		int n = 0;
		for (int j = 0; j < getProperty_secondary_coordinates().size(); ++j) {
			if (name == get_secondary_coordinates(j)) n++;
		}
		OPENSIM_THROW_IF(n>1, Exception, name + "listed multiple times in secondary_coordinates")
	}

	//Count numbers
	_n_prescribed_coord = getProperty_prescribed_coordinates().size();
	_n_primary_coord = getProperty_primary_coordinates().size();
	_n_secondary_coord = getProperty_secondary_coordinates().size();

	_prescribed_coord_name.setSize(_n_prescribed_coord);
	_prescribed_coord_path.setSize(_n_prescribed_coord);
	_prescribed_coord_index.setSize(_n_prescribed_coord);

	_primary_coord_name.setSize(_n_primary_coord);
	_primary_coord_path.setSize(_n_primary_coord);
	_primary_coord_index.setSize(_n_primary_coord);

	_secondary_coord_name.setSize(_n_secondary_coord);
	_secondary_coord_path.setSize(_n_secondary_coord);
	_secondary_coord_index.setSize(_n_secondary_coord);

	for (int i = 0; i < _n_prescribed_coord; ++i) {
		_prescribed_coord_path[i] = get_prescribed_coordinates(i);
		_prescribed_coord_name[i] = _model.getComponent<Coordinate>(get_prescribed_coordinates(i)).getName();
	}

	for (int i = 0; i < _n_primary_coord; ++i) {
		_primary_coord_path[i] = get_primary_coordinates(i);
		_primary_coord_name[i] = _model.getComponent<Coordinate>(get_primary_coordinates(i)).getName();
	}

	for (int i = 0; i < _n_secondary_coord; ++i) {
		_secondary_coord_path[i] = get_secondary_coordinates(i);
		_secondary_coord_name[i] = _model.getComponent<Coordinate>(get_secondary_coordinates(i)).getName();
	}

	int nCoord = 0;
	for (Coordinate& coord : _model.updComponentList<Coordinate>()) {
		std::string path = coord.getAbsolutePathString();

		int ind = _prescribed_coord_path.findIndex(path);
		if ( ind > -1){
			_prescribed_coord_index[ind] = nCoord;
		}

		ind = _primary_coord_path.findIndex(path);
		if (ind > -1) {
			_primary_coord_index[ind] = nCoord;
		}

		ind = _secondary_coord_path.findIndex(path);
		if (ind > -1) {
			_secondary_coord_index[ind] = nCoord;
		}
		nCoord++;
	}

	//Check for Secondary Damping
    _secondary_coord_damping.resize(_n_secondary_coord);
    _secondary_coord_damping = 0;

	for (int i = 0; i < _n_secondary_coord; ++i) {
		bool found = false;
		for (SpringGeneralizedForce& sgf : _model.updComponentList<SpringGeneralizedForce>()) {
			if (_secondary_coord_name[i] == sgf.get_coordinate()) {
				found = true;
                _secondary_coord_damping[i] = sgf.get_viscosity();                
			}
		}
		if (found == false) {
			std::cout << "Warning:: secondary_coord (" << _secondary_coord_name[i] <<
				") has no SpringGeneralizedForce, no damping applied!" << std::endl;
		}
	}

	//Append reserve actuators
	//if (get_primary_reserve_actuator_strength() != -1) {
		for (int i = 0; i < _n_primary_coord; ++i) {
			Coordinate& coord = _model.updComponent<Coordinate>(_primary_coord_path[i]);
			CoordinateActuator* res_act = new CoordinateActuator(coord.getName());
			//res_act->setOptimalForce(get_primary_reserve_actuator_strength());
            res_act->setOptimalForce(1);
            res_act->setMaxControl(100);
            res_act->setMinControl(-100);
			res_act->setName(coord.getName() + "_reserve");
			_model.addComponent(res_act);
		}
	//}

	
	for (int i = 0; i < _n_secondary_coord; ++i) {
		Coordinate& coord = _model.updComponent<Coordinate>(_secondary_coord_path[i]);
        CoordinateActuator* res_act = new CoordinateActuator(coord.getName());
		/*if (get_secondary_reserve_actuator_strength() != -1) {
			res_act->setOptimalForce(get_secondary_reserve_actuator_strength());
		}
		else {*/
		res_act->setOptimalForce(1);
		//}
		res_act->setName(coord.getName() + "_reserve");
        res_act->setMaxControl(100);
        res_act->setMinControl(-100);
		_model.addComponent(res_act);
			
	}
	

	//Compute Optimal Forces For Actuators
	_n_actuators = 0;
    _n_muscles = 0;
    _n_reserve_actuators = 0;
	for (ScalarActuator &actuator : _model.updComponentList<ScalarActuator>()) {
		_n_actuators++;
	}
	_optimal_force.resize(_n_actuators);

	int i = 0;
    for (const Muscle& msl : _model.updComponentList<Muscle>()) {
        //_optimal_force[i] = msl.getMaxIsometricForce();
        _optimal_force[i] = msl.getMaxIsometricForce()*cos(msl.getPennationAngleAtOptimalFiberLength()*SimTK::Pi/180);
        std::cout << msl.getName() << _optimal_force[i] << std::endl;
        i++;
        _n_muscles++;
    }
	for (CoordinateActuator &reserve : _model.updComponentList<CoordinateActuator>()) {
		_optimal_force[i] = reserve.getOptimalForce();
        std::cout << reserve.getName() << _optimal_force[i] << std::endl;
        i++;
        _n_reserve_actuators++;
	}

    _muscle_volumes = computeMuscleVolumes();

    if (true) {
        int w = 15;
        std::cout << "\nMuscle Properties" << std::endl;
        std::cout << std::setw(15) << "name " << std::setw(15) << "Fmax" << std::setw(15) << "Volume" << std::endl;
        i = 0;
        for (const Muscle& msl : _model.getComponentList<Muscle>()) {
            double l0 = msl.get_optimal_fiber_length();
            double fmax = msl.get_max_isometric_force();           

            std::cout << std::setw(15) << msl.getName() <<  std::setw(15) << fmax << std::setw(15) << _muscle_volumes[i] << std::endl;
            i++;
        }
    }

    _state = _model.initSystem();
}

/** ===================================================================
* RUN
* =====================================================================
*
*/

void COMAKTool::run()
{
	//Secondary Constraint Simulation
	if (get_perform_secondary_constraint_sim()) {
		performIKSecondaryConstraintSimulation();
	}

	//Inverse Kinematics 
	if (get_perform_inverse_kinematics()) {
		performIK();
	}



	//COMAK
	performCOMAK();
}

/** ===================================================================
* performIKSecondaryConstraintSimulation
* =====================================================================
*
*/

void COMAKTool::performIKSecondaryConstraintSimulation() {
	std::cout << "Performing IK Secondary Constraint Simulation..." << std::endl;
	
	Model model = _model;
	model.initSystem();

	//Setup Prescribed Function
	double start_time = get_secondary_constraint_sim_settle_time();
	double stop_time = start_time + get_secondary_constraint_sim_time();
	double t[] = { 0, start_time, stop_time };

	double start_value;
	double stop_value;

	if (model.getComponent<Coordinate>(get_secondary_coupled_coord()).getMotionType() == Coordinate::MotionType::Rotational) {
		start_value = get_secondary_coupled_coord_start_value() * SimTK::Pi / 180;
		stop_value = get_secondary_coupled_coord_stop_value() * SimTK::Pi / 180;
	}
	else {
		start_value = get_secondary_coupled_coord_start_value();
		stop_value = get_secondary_coupled_coord_stop_value();
	}
	
	double x[] = { start_value, start_value, stop_value };
	PiecewiseLinearFunction func(3, t, x);

	//Set coordinate Types
	for (auto& coord : model.updComponentList<Coordinate>()) {
		if (getProperty_secondary_coordinates().findIndex(coord.getAbsolutePathString()) > -1) {
			coord.set_locked(false);
		}
		else if (coord.getAbsolutePathString() == get_secondary_coupled_coord()){
			coord.set_locked(false);
			coord.set_prescribed(true);
			coord.set_prescribed_function(func);
			//coord.set_prescribed_function(get_secondary_coupled_coord_trajectory());
		}
		else {
			coord.set_locked(true);
		}		
	}

	for(auto& cc_const : model.updComponentList<CoordinateCouplerConstraint>()){
		std::string cc_coord_name = cc_const.getDependentCoordinateName();
		Coordinate& coord = model.updCoordinateSet().get(cc_coord_name);
		coord.set_locked(false);
	}

	// Set Muscle Properties
	//Add 2% muscle activation
/*	PrescribedController* msl_control = new PrescribedController();
	msl_control->setActuators(model.updActuators());

	for (Muscle& msl : model.updComponentList<Muscle>()) {
		msl_control->prescribeControlForActuator(msl.getName(), new Constant(0.05));
		if (msl.getConcreteClassName() == "Millard2012EquilibriumMuscle") {
			msl.set_ignore_activation_dynamics(true);
			msl.set_ignore_tendon_compliance(true);
		}
	}
	model.addComponent(msl_control);*/
	
		//Apply secondary damping multiplier
	if (get_settle_sim_viscosity_multiplier() != -1) {
		for (int i = 0; i < _n_secondary_coord; ++i) {
			bool found = false;
			for (SpringGeneralizedForce& sgf : model.updComponentList<SpringGeneralizedForce>()) {
				if (_secondary_coord_name[i] == sgf.get_coordinate()) {
					sgf.set_viscosity(sgf.get_viscosity()*get_settle_sim_viscosity_multiplier());
					found = true;
				}				
			}
			if (found ==false) {
				std::cout << "Warning:: secondary_coord (" << _secondary_coord_name[i] <<
					") has no SpringGeneralizedForce, no damping applied!" << std::endl;
			}
		}
	}

	//Initialize Model
	SimTK::State state = model.initSystem();
	model.equilibrateMuscles(state);

	//Prescribe Muscle Force
	for (Muscle& msl : model.updComponentList<Muscle>()) {
		msl.overrideActuation(state, true);
		double value = msl.getMaxIsometricForce()*0.01;
		msl.setOverrideActuation(state, value);
	}

	if (get_use_visualizer()) {
		SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
		viz.setBackgroundColor(SimTK::White);
		viz.setShowSimTime(true);
	}

	//Simulate
	double initialTime = 0.0;
	double finalTime = stop_time;

	TimeSeriesTable q_table;
	TimeSeriesTable u_table;

	SimTK::RowVector q_row(model.getNumCoordinates());
	SimTK::RowVector u_row(model.getNumCoordinates());

	std::vector<std::string> q_names;
	std::vector<std::string> u_names;

	for (Coordinate& coord : _model.updComponentList<Coordinate>()) {
		q_names.push_back(coord.getAbsolutePathString() + "/value");
		u_names.push_back(coord.getAbsolutePathString() + "/speed");
	}

	q_table.setColumnLabels(q_names);
	u_table.setColumnLabels(u_names);

	SimTK::CPodesIntegrator integrator(model.getSystem(), SimTK::CPodes::BDF, SimTK::CPodes::Newton);
	integrator.setAccuracy(get_secondary_constraint_sim_integrator_accuracy());
	
	SimTK::TimeStepper timestepper(model.getSystem(), integrator);

	timestepper.initialize(state);

	double dt = 0.01;

	int nSteps = round((finalTime - initialTime) / dt);

	for (int i = 0; i <= nSteps; ++i) {

		timestepper.stepTo(i*dt);
		state = timestepper.getState();

		int j = 0;
		for (const auto& coord : model.getComponentList<Coordinate>()) {
			q_row(j) = coord.getValue(state);
			u_row(j) = coord.getSpeedValue(state);
			j++;
		}
		q_table.appendRow(state.getTime(), q_row);
		u_table.appendRow(state.getTime(), u_row);
	}

	//Compute Coupled Constraint Functions
	std::vector<double> time = q_table.getIndependentColumn();
	int start_frame = round((start_time - initialTime) / dt);
	int nCutFrames = nSteps - start_frame;

	SimTK::Vector ind_data = q_table.getDependentColumn(get_secondary_coupled_coord() + "/value");
	SimTK::Vector cut_ind_data(nCutFrames);
	SimTK::Matrix cut_data(nCutFrames, _n_secondary_coord);

	int k = 0;
	for (int i = start_frame; i < nSteps; ++i) {
		cut_ind_data(k) = ind_data(i);
		k++;
	}

	for (int j = 0; j < _n_secondary_coord; ++j) {
		std::string path = _secondary_coord_path[j];
		SimTK::Vector data = q_table.getDependentColumn(path + "/value");

		k = 0;
		for (int i = start_frame; i < nSteps; ++i) {
			cut_data(k, j) = data(i);
			k++;
		}
	}
		
	
	
	double ind_max = SimTK::max(cut_ind_data);
	double ind_min = SimTK::min(cut_ind_data);
	
	int npts = 15;
	double step = (ind_max - ind_min) / npts;
	
	SimTK::Vector ind_pt_data(npts);

	for (int i = 0; i < npts; ++i) {
		ind_pt_data(i) = ind_min + i * step;
	}


	upd_secondary_constraint_funcs().clearAndDestroy();

	for (int j=0; j < _n_secondary_coord; ++j){
		std::string path = _secondary_coord_path[j];
		
		SimTK::Vector data = cut_data(j);

		//GCVSpline data_fit = GCVSpline(5, data.size(), &ind_data[0], &data[0]);
		SimmSpline data_fit = SimmSpline(data.size(), &cut_ind_data[0], &data[0]);

		
		SimmSpline* spline = new SimmSpline();
		spline->setName(path);		

		for (int i = 0; i < npts; ++i) {
			spline->addPoint(ind_pt_data(i), data_fit.calcValue(SimTK::Vector(1, ind_pt_data(i))));
		}
		
		upd_secondary_constraint_funcs().adoptAndAppend(spline);
	}

	//Write Outputs
	if (get_print_secondary_constraint_sim_results()) {
		q_table.addTableMetaData("inDegrees", std::string("no"));
		u_table.addTableMetaData("inDegrees", std::string("no"));
		_model.getSimbodyEngine().convertRadiansToDegrees(q_table);
		_model.getSimbodyEngine().convertRadiansToDegrees(u_table);
		
		std::string name = "secondary_constraint_sim_values";
		q_table.addTableMetaData("header", name);
		q_table.addTableMetaData("nRows", std::to_string(q_table.getNumRows()));
		q_table.addTableMetaData("nColumns", std::to_string(q_table.getNumColumns()+1));

		name = "secondary_constraint_sim_speeds";
		u_table.addTableMetaData("header", name);
		u_table.addTableMetaData("nRows", std::to_string(u_table.getNumRows()));
		u_table.addTableMetaData("nColumns", std::to_string(u_table.getNumColumns()+1));

		static const std::string q_mot_file{ get_results_dir() + "/secondary_constraint_sim_values.sto" };
		static const std::string u_mot_file{ get_results_dir() + "/secondary_constraint_sim_speeds.sto" };

		STOFileAdapter sto_file_adapt;
		sto_file_adapt.write(q_table, q_mot_file);
		sto_file_adapt.write(u_table, u_mot_file);
	}

	print(get_comak_settings_file());
}
/** ===================================================================
* PERFORM IK
* =====================================================================
*
*/
void COMAKTool::performIK() 
{
	Model model = _model;
	model.initSystem();

	for (int i = 0; i < getProperty_secondary_coordinates().size(); ++i) {
		std::string name = get_secondary_coordinates(i);
		std::string coord_name = model.getComponent<Coordinate>(name).getName();
		std::string ind_coord_name = model.getComponent<Coordinate>(get_secondary_coupled_coord()).getName();

		CoordinateCouplerConstraint* cc_constraint = new CoordinateCouplerConstraint();

		cc_constraint->setIndependentCoordinateNames(Array<std::string>(ind_coord_name, 1, 2));
		cc_constraint->setDependentCoordinateName(coord_name);
		cc_constraint->setFunction(get_secondary_constraint_funcs().get(name));
		cc_constraint->setName(coord_name + "_function");

		model.addComponent(cc_constraint);
	}
	if (get_print_ik_model()) {
		model.print(get_ik_print_model_file());
	}

	//SimTK::State state = model.initSystem();

	InverseKinematicsTool ik_tool(get_ik_settings_file(), false);
	ik_tool.setModel(model);
	ik_tool.setMarkerDataFileName(get_trc_kinematics_file());
	ik_tool.setOutputMotionFileName(get_ik_motion_file());
	ik_tool.setResultsDir(get_results_dir());

	/*if (get_use_visualizer()) {
		SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
		viz.setBackgroundColor(SimTK::White);
		viz.setShowSimTime(true);
	}*/

	ik_tool.run();
}
/** ===================================================================
* equilibriateSecondaryCoordinates
* =====================================================================
*
*/

SimTK::Vector COMAKTool::equilibriateSecondaryCoordinates() 
{
	Model model = _model;
	SimTK::State state = model.initSystem();

	std::cout << std::endl;
	std::cout << "------------------------------------------------------------------" << std::endl;
	std::cout << "Performing forward simulation to equilibriate secondary kinematics" << std::endl;
	std::cout << "------------------------------------------------------------------" << std::endl;
	std::cout << "Secondary Damping Multiplier: " << get_settle_sim_viscosity_multiplier() << std::endl;

	if (get_use_visualizer()) {
		SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
		viz.setBackgroundColor(SimTK::White);
		viz.setShowSimTime(true);
	}
	
	
	//Apply secondary damping multiplier
	if (get_settle_sim_viscosity_multiplier() != -1) {
		for (int i = 0; i < _n_secondary_coord; ++i) {
			bool found = false;
			for (SpringGeneralizedForce& sgf : model.updComponentList<SpringGeneralizedForce>()) {
				if (_secondary_coord_name[i] == sgf.get_coordinate()) {
					sgf.set_viscosity(sgf.get_viscosity()*get_settle_sim_viscosity_multiplier());
					found = true;
				}				
			}
			if (found ==false) {
				std::cout << "Warning:: secondary_coord (" << _secondary_coord_name[i] <<
					") has no SpringGeneralizedForce, no damping applied!" << std::endl;
			}
		}
	}

	//Add 2% muscle activation
	/*PrescribedController* msl_control = new PrescribedController();
	msl_control->setActuators(model.updActuators());
*/
	for (Muscle& msl : model.updComponentList<Muscle>()) {
		//msl_control->prescribeControlForActuator(msl.getName(), new Constant(0.05));
		if (msl.getConcreteClassName() == "Millard2012EquilibriumMuscle") {
			msl.set_ignore_activation_dynamics(true);
			msl.set_ignore_tendon_compliance(true);
		}
	}
	//model.addComponent(msl_control);*/

	state = model.initSystem();
	


	//Set initial pose and Lock all coordinates except secondary 
	int nCoord = 0;
	for (Coordinate& coord : model.updComponentList<Coordinate>()) {

		coord.setValue(state, _q_matrix(_start_frame, nCoord), false);
		coord.setSpeedValue(state, _u_matrix(_start_frame, nCoord));
		
		if (_secondary_coord_path.findIndex(coord.getAbsolutePathString()) > -1) {
			coord.setLocked(state, false);
		}
		else {
			coord.setLocked(state, true);
		}
		nCoord++;
	}

	for(auto& cc_const : model.updComponentList<CoordinateCouplerConstraint>()){
		std::string cc_coord_name = cc_const.getDependentCoordinateName();
		Coordinate& coord = model.updCoordinateSet().get(cc_coord_name);
		coord.setLocked(state,false);
	}

	model.assemble(state);
	model.realizeVelocity(state);

		//Prescribe Muscle Force
	for (Muscle& msl : model.updComponentList<Muscle>()) {
		msl.overrideActuation(state, true);
		double value = msl.getMaxIsometricForce()*0.02;
		std::cout << msl.getName() << " " << value << std::endl;
		msl.setOverrideActuation(state, value);
	}
	
	model.realizeAcceleration(state);

	//Setup results storage
	TimeSeriesTable q_table;
	TimeSeriesTable u_table;

	SimTK::RowVector q_row(model.getNumCoordinates());
	SimTK::RowVector u_row(model.getNumCoordinates());

	std::vector<std::string> q_names;
	std::vector<std::string> u_names;

	for (auto coord : model.getComponentList<Coordinate>()) {
		q_names.push_back(coord.getName() + "/value");
		u_names.push_back(coord.getName() + "/speed");
	}

	q_table.setColumnLabels(q_names);
	u_table.setColumnLabels(u_names);



	//Store Secondary Coordinate Values
	SimTK::Vector prev_sec_coord_value(_n_secondary_coord);

	for (int k = 0; k < _n_secondary_coord; k++) {
		Coordinate& coord = model.updComponent<Coordinate>(_secondary_coord_path[k]);
		prev_sec_coord_value(k) = coord.getValue(state);
	}

	//Integrate 
	SimTK::CPodesIntegrator integrator(model.getSystem(), SimTK::CPodes::BDF, SimTK::CPodes::Newton);
	integrator.setAccuracy(get_settle_accuracy());

	SimTK::TimeStepper timestepper(model.getSystem(), integrator);
	timestepper.initialize(state);
	
	double dt = 0.01;

	double max_coord_delta = SimTK::Infinity;
	int i = 1;
	while (max_coord_delta > get_settle_threshold()){


		//model.realizeAcceleration(s);
		timestepper.stepTo(i*dt);
		state = timestepper.getState();
		
		if (get_verbose() > 0) {
			std::cout << std::endl;
			std::cout << "Time: " << state.getTime() << std::endl;
			std::cout << "\t\t VALUE \t\tDELTA" << std::endl;
		}

		//Record Results
		int j = 0;
		for (const auto& coord : model.getComponentList<Coordinate>()) {
			q_row(j) = coord.getValue(state);
			u_row(j) = coord.getSpeedValue(state);
			j++;
		}
		q_table.appendRow(state.getTime(), q_row);
		u_table.appendRow(state.getTime(), u_row);

		//Compute Delta Coordinate
		max_coord_delta = 0;
		for (int k = 0; k < _n_secondary_coord; k++) {
			Coordinate& coord = model.updComponent<Coordinate>(_secondary_coord_path[k]);
			double value = coord.getValue(state);
			double delta = abs(value - prev_sec_coord_value(k));
			
			if (delta > max_coord_delta) {
				max_coord_delta = delta;
			}
			prev_sec_coord_value(k) = value;

			if (get_verbose() > 0) {
				std::cout << coord.getName() << " \t" << value << "\t" << delta <<std::endl;
			}
		}
		i++;
	}
	
	//Print Results
	if (get_print_settle_sim_results()) {
		std::string name = "settle_sim_values";
		q_table.addTableMetaData("header", name);
		q_table.addTableMetaData("nRows", std::to_string(q_table.getNumRows()));
		q_table.addTableMetaData("nColumns", std::to_string(q_table.getNumColumns()+1));

		name = "settle_sim_speeds";
		u_table.addTableMetaData("header", name);
		u_table.addTableMetaData("nRows", std::to_string(u_table.getNumRows()));
		u_table.addTableMetaData("nColumns", std::to_string(u_table.getNumColumns()+1));
		
		
		STOFileAdapter sto_adapt;
		sto_adapt.write(q_table, get_settle_sim_results_dir() + "/" + get_settle_sim_results_prefix() + "_settle_sim_values.sto");
		sto_adapt.write(u_table, get_settle_sim_results_dir() + "/" + get_settle_sim_results_prefix() + "_settle_sim_speeds.sto");	
	}

	SimTK::Vector secondary_q(_n_secondary_coord);
	
	//Collect Settled Secondary Q values;
	for (int i = 0; i < _n_secondary_coord; ++i) {
		Coordinate& coord = model.updComponent<Coordinate>(_secondary_coord_path[i]);
		secondary_q(i) = coord.getValue(state);
	}
	return secondary_q;
}

/** ===================================================================
* PERFORM COMAK
* =====================================================================
*
*/
void COMAKTool::performCOMAK()
{
	printCOMAKascii();

	SimTK::State state = _model.initSystem();

	//Read Kinematics and Compute Desired Accelerations
	extractKinematicsFromFile();

	//Initialize Secondary Kinematics
	SimTK::Vector init_secondary_values(_n_secondary_coord);
	
	if (get_equilibriate_secondary_coordinates_at_start()) {
		init_secondary_values = equilibriateSecondaryCoordinates();
	}
	else {
		for (int i = 0; i < _n_secondary_coord; ++i) {
			init_secondary_values(i) = _q_matrix(_start_frame, _secondary_coord_index[i]);
		}
	}

	if (get_verbose() > 1) {
		std::cout << std::endl;
		std::cout << "Initial Secondary Coordinate Values:" << std::endl;
		
		for (int i = 0; i < _n_secondary_coord; ++i) {
			std::cout << _secondary_coord_name[i] << ":\t" << init_secondary_values(i) << std::endl;
		}
		std::cout << std::endl;
	}

	//Apply Damping Multiplier
	if (get_comak_damping_multiplier() != -1) {
		for (int i = 0; i < _n_secondary_coord; ++i) {
			bool found = false;
			for (SpringGeneralizedForce& sgf : _model.updComponentList<SpringGeneralizedForce>()) {
				if (_secondary_coord_name[i] == sgf.get_coordinate()) {
					sgf.set_viscosity(sgf.get_viscosity()*get_comak_damping_multiplier());
					found = true;
				}				
			}
			if (found ==false) {
				std::cout << "Warning:: secondary_coord (" << _secondary_coord_name[i] <<
					") has no SpringGeneralizedForce, no damping applied!" << std::endl;
			}
		}
	}
	//Apply External Loads
	applyExternalLoads();

	//Set Prescribed Coordinates
	int nCoord = 0;
	for (Coordinate& coord : _model.updComponentList<Coordinate>()) {
		if (getProperty_prescribed_coordinates().findIndex(coord.getAbsolutePathString()) > -1) {
			SimTK::Vector data = _q_matrix(nCoord);
			GCVSpline func = GCVSpline(5, _n_frames, &_time[0], &data[0], coord.getName());
			coord.set_prescribed_function(func);
			coord.set_prescribed(true);			
		}
		nCoord++;
	}

	for(auto& cc_const : _model.updComponentList<CoordinateCouplerConstraint>()){
		std::string cc_coord_name = cc_const.getDependentCoordinateName();
		Coordinate& coord = _model.updCoordinateSet().get(cc_coord_name);
		coord.set_locked(false);
		coord.set_prescribed(false);
	}

	state = _model.initSystem();

	//Results Storage
    StatesTrajectory states_trajectory;

	std::vector<std::string> actuator_names;
    
    for (Muscle &msl : _model.updComponentList<Muscle>()) {
		actuator_names.push_back(msl.getName());		
	}
	for (CoordinateActuator &reserve : _model.updComponentList<CoordinateActuator>()) {
		actuator_names.push_back(reserve.getName());		
	}

	TimeSeriesTable activations_table;
	activations_table.setColumnLabels(actuator_names);

	TimeSeriesTable forces_table;
	forces_table.setColumnLabels(actuator_names);

	std::vector<std::string> kinematics_names;

	for (Coordinate &coord : _model.updComponentList<Coordinate>()) {
		std::string name = coord.getAbsolutePathString();
		kinematics_names.push_back(name + "/value");
		kinematics_names.push_back(name + "/speed");
		kinematics_names.push_back(name + "/acc");
	}

	TimeSeriesTable kinematics_table;
	kinematics_table.setColumnLabels(kinematics_names);

	//Prepare for Optimization
	_model.setAllControllersEnabled(false);

	for (ScalarActuator &actuator : _model.updComponentList<ScalarActuator>()) {
		actuator.overrideActuation(state, true);
	}
	
	//parameters vector ordered by activations then secondary coord values
	SimTK::Vector parameters(_n_actuators + _n_secondary_coord);
	parameters = 0;
	for (int i = 0; i < _n_muscles; ++i) {
		parameters[i] = 0.02;
	}
    for (int i = 0; i < _n_secondary_coord; ++i) {
		parameters[i + _n_actuators] = init_secondary_values(i);
	}

	_n_parameters = parameters.size();
	_prev_parameters = parameters;

	
	_parameter_names.setSize(_n_parameters);

	int p = 0;
    for (Muscle &msl : _model.updComponentList<Muscle>()) {
		_parameter_names[p] = msl.getName();
		p++;
	}
	for (CoordinateActuator &reserve : _model.updComponentList<CoordinateActuator>()) {
		_parameter_names[p] = reserve.getName();
		p++;
	}
	for (int p = 0; p < _n_secondary_coord; ++p) {
		_parameter_names[_n_actuators + p] = _secondary_coord_name[p];
	}

	//Set initial Secondary Qs
	_dt = _time[1] - _time[0];

	_prev_secondary_value.resize(_n_secondary_coord);

	for (int j = 0; j < _n_secondary_coord; ++j) {
		Coordinate& coord = _model.updComponent<Coordinate>(_secondary_coord_path[j]);
		coord.setValue(state, init_secondary_values(j), false);		
        _prev_secondary_value(j) = init_secondary_values(j);
	}

	//Visualize
	SimTK::Visualizer* viz;
	if (get_use_visualizer()) {
		viz = &_model.updVisualizer().updSimbodyVisualizer();
		viz->setBackgroundColor(SimTK::White);
		viz->setShowSimTime(true);
		/*for (int i = 0; i < _external_loads.getSize(); ++i) {
			viz->addDecorationGenerator(new ForceArrowGenerator(_model, _external_loads[i]));
		}*/		
	}

	_consecutive_bad_frame = 1;

	//Loop over each time step
	//------------------------
	int frame_num = 0;
	for (int i = 0; i < _n_frames; ++i) {
		if (_time[i] < get_start_time()) { continue; }
		if (_time[i] > get_stop_time()) { break; };

		//Set Time
		state.setTime(_time[i]);

		std::cout << "================================================================================" << std::endl;
		std::cout << "Frame: " << ++frame_num << "/" << _n_out_frames << std::endl;
		std::cout << "Time: " << _time[i] << std::endl;
		std::cout << "================================================================================\n" << std::endl;

		//Set Primary Qs and Us to experimental values
        for (int j = 0; j < _n_primary_coord; ++j) {
		    Coordinate& coord = _model.updComponent<Coordinate>(_primary_coord_path[j]);
			coord.setValue(state, _q_matrix(i, _primary_coord_index[j]),false);
			coord.setSpeedValue(state, _u_matrix(i, _primary_coord_index[j]));
	    }

        //Reset the Secondary Us to experimental values
        for (int j = 0; j < _n_secondary_coord; ++j) {
		    Coordinate& coord = _model.updComponent<Coordinate>(_secondary_coord_path[j]);			
			coord.setSpeedValue(state, _u_matrix(i, _secondary_coord_index[j]));
	    }
        
		_model.assemble(state);
		_model.realizeVelocity(state);

        //Check Contact Forces
        if (false) {
            _model.realizeAcceleration(state);
            
            /*std::cout << "Initial Contact Forces" << std::endl;
            for (auto& frc : _model.updComponentList<Smith2018ArticularContactForce>()) {
                SimTK::Vec3 frc_vec = frc.getContactStatsTotalVec3(state, "casting_mesh.total.contact_force");

                std::cout << frc.getName() << "\t" << frc_vec[0] << "\t" << frc_vec[1] << "\t" << frc_vec[2] << std::endl;
            }*/

            std::cout << "Initial Contact Energy" << std::endl;
            for (Smith2018ArticularContactForce& cnt_frc : _model.updComponentList<Smith2018ArticularContactForce>()) {
                std::cout << cnt_frc.getName() << "\t" << cnt_frc.getOutputValue<double>(state,"potential_energy") << std::endl;        
            }

            std::cout << "Initial Muscle Forces" << std::endl;
            for (auto& msl : _model.updComponentList<Muscle>()) {
                double frc = msl.getActuation(state);

                std::cout << msl.getName() << "\t" << frc << std::endl;
            }

            /*std::cout << std::setw(15) << "Initial Ligament" << std::setw(15) <<"Total_Force" << std::setw(15) <<"Spring_Force" << std::setw(15) <<"Damping_Force"<< std::setw(15)  << "Lengths" << std::setw(15) << "Strains" << std::setw(15) << "Slack Length" << std::setw(15) << "Reference Length" << std::endl;
            for (auto& lig : _model.updComponentList<Blankevoort1991Ligament>()) {
                double total_frc = lig.getDynamicQuantities(state,"force_total");
                double spring_frc = lig.getDynamicQuantities(state,"force_spring");
                double damping_frc = lig.getDynamicQuantities(state,"force_damping");
                double length = lig.getDynamicQuantities(state, "length");
                double strain  = lig.getDynamicQuantities(state, "strain");
                double slack_length = lig.getCacheVariableValue<double>(state, "slack_length");
                double ref_length = lig.getCacheVariableValue<double>(state, "reference_length");
                std::cout << std::setw(15) << lig.getName() << std::setw(15) << total_frc << std::setw(15) << spring_frc << std::setw(15) << damping_frc << std::setw(15) << length << std::setw(15) << strain << std::setw(15) << slack_length << std::setw(15) << ref_length << std::endl;
            }*/


        }
		//Iterate for COMAK Solution		
		double max_udot_error = SimTK::Infinity;
		SimTK::Vector iter_max_udot_error(get_max_iterations(),0.0);
        std::vector<std::string> iter_max_udot_coord(get_max_iterations(), "");
		SimTK::Matrix iter_parameters(get_max_iterations(), _n_parameters,0.0);

		int iter;
        for (iter = 0; iter < get_max_iterations(); ++iter) {

            std::cout << std::endl;
            std::cout << "--------------------------------------------------------------------------------" << std::endl;
            std::cout << "Time: " << _time[i] << std::endl;
            std::cout << "iteration: " << iter << std::endl;
            std::cout << "--------------------------------------------------------------------------------" << std::endl;

            if (get_verbose() > 2) {
                std::cout << "Initial Coordinate Values Speeds" << std::endl;
                for (Coordinate& coord : _model.updComponentList<Coordinate>()) {
                    std::cout << std::setw(20) << coord.getName() << std::setw(15) << coord.getValue(state) << std::setw(15) << coord.getSpeedValue(state) << std::endl;
                }
            }

            //Reset the reserve actuators       
            /*int j = _n_muscles;
            for (CoordinateActuator &coord_act : _model.updComponentList<CoordinateActuator>()) {
		        coord_act.overrideActuation(state, true);
		        parameters[j] = 0;
		        coord_act.setOverrideActuation(state,0);		
                j++;
            }*/

            ComakTarget target = ComakTarget(state, &_model, ~_udot_matrix[i], parameters, _primary_coord_path, _secondary_coord_path, false);
            /*
            SimTK::Vector weight(_n_actuators);
            SimTK::Vector desired_act(_n_actuators);

            int j = 0;
            for (auto& ca : updComponentList<CoordinationActuator>()) {
            weight(j) = ca.get_cost_function_weight().calcValue(SimTK::Vector(1, _time[i]));
            desired_act(j) = ca.get_desired_activation().calcValue(SimTK::Vector(1, _time[i]));
            j++;
            }
            std::cout << "Weight: " << weight << std::endl;
            std::cout << "DA: " << desired_act << std::endl;

            target.setCostFunctionWeight(weight);
            target.setDesiredActivation(desired_act);
            */

            target.setMaxChangeRotation(get_max_change_rotation());
            target.setMaxChangeTranslation(get_max_change_translation());
            target.setUdotTolerance(get_udot_tolerance());
            target.setUnitUdotEpsilon(get_unit_udot_epsilon());
            target.setDT(_dt);
            target.setOptimalForces(_optimal_force);
            //target.setObservedSpeeds(~_u_matrix[i]);
            //target.setSecondaryIndex(_secondary_coord_index);
            target.setPrevSecondaryValues(_prev_secondary_value);
            target.setMuscleVolumes(_muscle_volumes);
            target.setSecondaryCoordinateDamping(_secondary_coord_damping);
            target.setContactEnergyWeight(get_contact_energy_weight());
            //target.setComakDampingMultiplier(get_comak_damping_multiplier());
            target.initialize();

            //SimTK::OptimizerAlgorithm algorithm = SimTK::InteriorPoint;
            SimTK::OptimizerAlgorithm algorithm = SimTK::CFSQP;
            SimTK::Optimizer optimizer(target, algorithm);

            optimizer.setDiagnosticsLevel(0);

            optimizer.setMaxIterations(5000);
            optimizer.setConvergenceTolerance(0.00000001);
            
            optimizer.useNumericalGradient(false);
            optimizer.useNumericalJacobian(false);

            if (algorithm == SimTK::InteriorPoint) {
                // Some IPOPT-specific settings
                //optimizer.setLimitedMemoryHistory(500); // works well for our small systems
                optimizer.setAdvancedBoolOption("warm_start", true);
                optimizer.setAdvancedRealOption("obj_scaling_factor", 1);
                optimizer.setAdvancedRealOption("nlp_scaling_max_gradient", 1);
            }

            //for (int m = 0; m < 10; ++m) {
                try {
                    optimizer.optimize(parameters);
                    if (false) {
                        std::cout << std::endl << std::endl;
                        std::cout << "Optimized Parameters:" << std::endl;
                        for (int p = 0; p < _n_parameters; p++) {
                            std::cout << _parameter_names[p] << " " << parameters[p] << std::endl;
                        }
                    }
                    //break;
                }
                catch (SimTK::Exception::Base ex) {
                /*    if (get_verbose() > 2) {
                        std::cout << std::endl << std::endl;
                        std::cout << "Optimized Parameters:" << std::endl;
                        for (int p = 0; p < _n_parameters; p++) {
                            std::cout << _parameter_names[p] << " " << parameters[p] << std::endl;
                        }
                    }
                    std::cout << "COMAK Optimization failed, upping the parameter bounds: " << ex.getMessage() << std::endl;

                    target.setParameterBounds(m);

                    optimizer.setOptimizerSystem(target);
*/
                }

           // }
            iter_parameters[iter] = ~parameters;
            
                        
            setStateFromComakParameters(state, parameters);

            _model.realizeAcceleration(state);

            //Output Optimization Results
            if (get_verbose() > 0) {           
                int w = 20;

                std::cout << std::left << "\nOptimized Muscles:" << std::endl;
                std::cout << std::setw(w) << "name" << std::setw(w) << "activation" << std::setw(w) << "force" << std::endl;
                int p = 0;
                for (int k = 0; k < _n_muscles; ++k) {
                    std::cout << std::setw(w) <<_parameter_names[p] << std::setw(w) << parameters[p] << std::setw(w) <<  parameters[p]*_optimal_force[p] <<std::endl;
                    p++;
                }

                std::cout << "\nOptimized Reserve Actuators:" << std::endl;
                std::cout << std::setw(w) << "name" << std::setw(w) << "activation" << std::setw(w) << "force" << std::endl;
                for (int k = 0; k < _n_reserve_actuators; ++k) {
                    std::cout << std::setw(w) << _parameter_names[p] << std::setw(w) << parameters[p] << std::setw(w) <<  parameters[p]*_optimal_force[p] <<std::endl;
                    p++;
                }

                std::cout << "\nOptimized Secondardy Coordinates:" << std::endl;
                std::cout << std::setw(w) << "name" << std::setw(w) << "value" << std::setw(w) << "change" << std::setw(w) << "damping force" << std::endl;
                for (int k = 0; k < _n_secondary_coord; ++k) {
                    double dq = parameters[p] - _prev_secondary_value[k];
                    double damping_frc = -dq / _dt * _secondary_coord_damping[k];
                    std::cout << std::setw(w) << _secondary_coord_name[k] << std::setw(w) << parameters[p] << std::setw(w) << dq << std::setw(w) << damping_frc << std::endl;
                    p++;
                }
                target.printPerformance(parameters);

                std::cout << "\nOptimized Ligaments:" << std::endl;
                /*std::cout << std::setw(15) << "Total_Force" << std::setw(15) << "Spring_Force" << std::setw(15) << "Damping_Force" << std::setw(15) << "Lengths" << std::setw(15) << "Strains" << std::endl;
                for (auto& lig : _model.updComponentList<Blankevoort1991Ligament>()) {
                    double total_frc = lig.getDynamicQuantities(state,"force_total");
                    double spring_frc = lig.getDynamicQuantities(state,"force_spring");
                    double damping_frc = lig.getDynamicQuantities(state,"force_damping");
                    double length = lig.getDynamicQuantities(state, "length");
                    double strain  = lig.getDynamicQuantities(state, "strain");
                    std::cout << std::setw(15) << lig.getName() << std::setw(15) << total_frc << std::setw(15) << spring_frc << std::setw(15) << damping_frc << std::setw(15) << length << std::setw(15) << strain << std::endl;
                }*/
            }

            //Compute udot linearized error
            if (get_verbose() > 1) {
                std::cout << "Acceleration Error Calculation" << std::endl;
                std::cout << "Initial Coordinate Values Speeds" << std::endl;
                for (Coordinate& coord : _model.updComponentList<Coordinate>()) {
                    std::cout << std::setw(20) << coord.getName() << std::setw(15) << coord.getValue(state) << std::setw(15) << coord.getSpeedValue(state) << std::endl;
                }
            }

            if (get_verbose() > 0) {
                std::cout << "\nOptimized Acceleration Errors:" << std::endl;
                std::cout << std::setw(20) << "Name" << std::setw(20) << "Experimental" << std::setw(20) << "Simulated" << std::setw(20) << "Error" << std::endl;
            }

            std::string max_udot_coord = "";
            int k = 0;
            max_udot_error = 0;
            for (const Coordinate& coord : _model.getComponentList<Coordinate>()) {
                std::string path = coord.getAbsolutePathString();


                double coord_udot = coord.getAccelerationValue(state);
                double udot_error;
                double observed_udot;
                if (_secondary_coord_path.findIndex(path) > -1) {
                    observed_udot = 0.0;                   
                }
                else {
                    observed_udot = _udot_matrix(i, k);
                }
                udot_error = abs(observed_udot - coord_udot);

                k++;
                if (_secondary_coord_path.findIndex(path) == -1 && _primary_coord_path.findIndex(path) == -1) {
                    continue;
                }

                if (udot_error > max_udot_error) {
                    max_udot_coord = coord.getName();
                    max_udot_error = udot_error;
                }

                if (get_verbose() > 0) {                   
                    std::cout << std::setw(20) << coord.getName() << std::setw(20) << observed_udot  << std::setw(20)  << coord_udot << std::setw(20) << udot_error << std::endl;
                }
            }

            std::cout << "\nMax udot Error: " << max_udot_error << std::endl;
            std::cout << "Max Error Coord: " << max_udot_coord << std::endl;
            iter_max_udot_error(iter) = max_udot_error;
            iter_max_udot_coord[iter] = max_udot_coord;

            if (false){
                for (SpringGeneralizedForce& sgf : _model.updComponentList<SpringGeneralizedForce>()) {
                    for (int m = 0; m < sgf.getRecordLabels().size(); ++m) {
                        std::cout << sgf.getRecordLabels().get(m) << ": " << sgf.getRecordValues(state).get(m) << std::endl;
                    }
                }
            }

			//Check for convergence
			if (max_udot_error < get_udot_tolerance()) {
				_consecutive_bad_frame=1; //converged so reset
				break;
			}
            //std::cin.ignore();
		}// END COMAK ITERATION

		if (iter == get_max_iterations()) {
			std::cout << std::endl;
			std::cout << "COMAK failed to converge." << std::endl;

			_consecutive_bad_frame++;

			//Reset to best iteration solution
			double min_val = get_udot_worse_case_tolerance();
			int min_iter = -1;
            std::string bad_coord;
			for (int m = 0; m < iter_max_udot_error.size(); ++m) {
				if (iter_max_udot_error(m) < min_val) {
					min_val = iter_max_udot_error(m);
					min_iter = m;
                    bad_coord = iter_max_udot_coord[m];
				}
			}
			if (min_iter > -1) {
				parameters = ~iter_parameters[min_iter];
				std::cout << "Using best iteration (" << min_iter << ") with max udot error: " << min_val << std::endl;
			}
			else {
				parameters = _prev_parameters;
				std::cout << "No iteration has max udot error less than worst case tolerance (" << get_udot_worse_case_tolerance() << ").\n"
					<< "Resetting optimization parameters to previous time step solution." << std::endl;
			}
			setStateFromComakParameters(state, parameters);

            //Save data about failed convergence
            _bad_frames.push_back(frame_num);
            _bad_times.push_back(_time[i]);
            _bad_udot_errors.push_back(min_val);
            _bad_udot_coord.push_back(bad_coord);
		}
		

		//Store Solution
		_model.realizeAcceleration(state);
		_prev_parameters = parameters;

		for (int m = 0; m < _n_secondary_coord; ++m) {
			Coordinate& coord = _model.updComponent<Coordinate>(_secondary_coord_path[m]);
			_prev_secondary_value(m) = coord.getValue(state);
		}

		//Save the results
        states_trajectory.append(state);

		SimTK::RowVector activations(_n_actuators);
		SimTK::RowVector forces(_n_actuators);
		

		for (int m = 0; m < _n_actuators; ++m) {
			activations(m) = parameters(m);
			forces(m) = parameters(m)*_optimal_force(m);
		}

		activations_table.appendRow(_time[i], activations);		
		forces_table.appendRow(_time[i], forces);

		SimTK::RowVector kinematics(_model.getNumCoordinates()*3);

		int k = 0;
		for (Coordinate& coord : _model.updComponentList<Coordinate>()) {
			kinematics(k) = coord.getValue(state);
			kinematics(k+1) = coord.getSpeedValue(state);
			kinematics(k+2) = coord.getAccelerationValue(state);
			k = k + 3;
		}
		kinematics_table.appendRow(_time[i], kinematics);

        //Perform Inverse Dynamics
        /*for (Muscle &msl : _model.updComponentList<Muscle>()) {
            msl.setActivation(state, 0);
        }
        auto id_solver = InverseDynamicsSolver(_model);
        SimTK::Vector gen_forces = id_solver.solve(state, state.getUDot());
        */
		//Visualize the Results
		if (get_use_visualizer()) {
			int k = 0;
			for (Muscle &msl : _model.updComponentList<Muscle>()) {
				msl.overrideActuation(state, false);
				_model.realizePosition(state);
				msl.setActivation(state, parameters[k]);
                k++;
		    }

			_model.realizeAcceleration(state);
			viz->drawFrameNow(state);

            for (Muscle &msl : _model.updComponentList<Muscle>()) {
                msl.overrideActuation(state, true);
            }
		}
	} //END of COMAK timestep

    //Print Convergence Summary
    std::cout << "Convergence Summary:" << std::endl;
    std::cout << "--------------------" << std::endl;
    std::cout << std::setw(15) << "Bad Times" << std::setw(15) << "Bad Frames" << std::setw(15) << "udot error" << std::endl;

    for (int i = 0; i < _bad_frames.size(); i++) {
        std::cout << std::setw(15) << _bad_times[i] << std::setw(15) << _bad_frames[i] << std::setw(15) << _bad_udot_errors[i] << std::endl;
    }

	//Print Results
    STOFileAdapter results_adapter;
    TimeSeriesTable states_table = states_trajectory.exportToTable(_model);
    results_adapter.write(states_table, get_results_dir() + get_results_prefix() + "_states.sto");

    activations_table.addTableMetaData("header", std::string("ModelActivations"));
	activations_table.addTableMetaData("nRows", std::to_string(_n_out_frames));
	activations_table.addTableMetaData("nColumns", std::to_string(_n_actuators + 1));

	results_adapter.write(activations_table, get_results_dir() + get_results_prefix() + "_activation.sto");

	
	forces_table.addTableMetaData("header", std::string("ModelForces"));
	forces_table.addTableMetaData("nRows", std::to_string(_n_out_frames));
	forces_table.addTableMetaData("nColumns", std::to_string(_n_actuators + 1));

	results_adapter.write(forces_table, get_results_dir() + get_results_prefix() + "_force.sto");

	kinematics_table.addTableMetaData("inDegrees", std::string("no"));
	_model.getSimbodyEngine().convertRadiansToDegrees(kinematics_table);
	kinematics_table.addTableMetaData("header", std::string("ModelKinematics"));
	kinematics_table.addTableMetaData("nRows", std::to_string(_n_out_frames));
	kinematics_table.addTableMetaData("nColumns", std::to_string(_model.getNumCoordinates()*3 + 1));

	results_adapter.write(kinematics_table, get_results_dir() + get_results_prefix() + "_kinematics.sto");
}

void COMAKTool::setStateFromComakParameters(SimTK::State& state, const SimTK::Vector parameters) {
	//Set Muscle Activations to Optimized
	int j = 0;
	for (Muscle &msl : _model.updComponentList<Muscle>()) {
		msl.overrideActuation(state, true);
		double force = _optimal_force[j] * parameters[j];
		msl.setOverrideActuation(state,force);		
        j++;
    }
    //Set Reserve Activations to Optimized
    for (CoordinateActuator &coord_act : _model.updComponentList<CoordinateActuator>()) {
		coord_act.overrideActuation(state, true);
		double force = _optimal_force[j] * parameters[j];
		coord_act.setOverrideActuation(state,force);		
        j++;
    }    
    _model.realizeAcceleration(state);



	//Set Secondary Kinematics to Optimized
	for (int m = 0; m < _n_secondary_coord; ++m) {
		Coordinate& coord = _model.updComponent<Coordinate>(_secondary_coord_path[m]);

		double value = parameters(_n_actuators + m);
		coord.setValue(state, value, false);

        double speed = (value - _prev_secondary_value(m)) / _dt;
        //double speed = 0;
		coord.setSpeedValue(state, speed);

        if (false) {
            std::cout << "Coordinate \tSpeed \tValue \tPrior Value" << std::endl;
            std::cout << coord.getName() << "\t" << speed << "\t" << value << "\t" << _prev_secondary_value(m) << std::endl;
        }
	}
	_model.assemble(state);
 }

void COMAKTool::extractKinematicsFromFile() {

	Storage store(get_ik_motion_file());
	Array<std::string> col_names = store.getColumnLabels();

	//Set Start and Stop Times
	Array<double> in_time;
	store.getTimeColumn(in_time);

	if (get_start_time() == -1) {
		set_start_time(in_time.get(0));
	}
	if (get_stop_time() == -1) {
		set_stop_time(in_time.getLast());
	}

	if (store.isInDegrees()) {
		_model.getSimbodyEngine().convertDegreesToRadians(store);
	}

	if (get_time_step() != -1) {
		store.resampleLinear(get_time_step());
	}

	if (get_lowpass_filter_frequency() != -1) {
		store.pad(store.getSize() / 2);
		store.lowpassIIR(get_lowpass_filter_frequency());
	}

	store.getTimeColumn(_time);

	//Set number of Frames
	_n_frames = _time.size();
	_n_out_frames = 0;
	bool first_frame = true;

	for (int i = 0; i < _n_frames; ++i) {
		if (_time[i] < get_start_time()) { continue; }
		if (_time[i] > get_stop_time()) { break; };

		if (first_frame) {
			_start_frame=i;
			first_frame = false;
		}

		_n_out_frames++;
	}

	//Gather Q and U values
	Array<std::string> col_labels = store.getColumnLabels();

	SimTK::Vector q_col_map(_model.getNumCoordinates());
	q_col_map = -1;

	for (int i = 0; i < col_labels.size(); ++i) {
		std::vector<std::string> split_label = split_string(col_labels[i], "/");

		int j = 0;
		for (const Coordinate& coord : _model.getComponentList<Coordinate>()) {
			if (contains_string(split_label, coord.getName())) {
				if (split_label.back() == "value" || split_label.back() == coord.getName()) {
					q_col_map(j) = i;
				}
			}
			j++;
		}

	}

	_q_matrix.resize(_n_frames, _model.getNumCoordinates());
	_u_matrix.resize(_n_frames, _model.getNumCoordinates());
	_udot_matrix.resize(_n_frames, _model.getNumCoordinates());

	_q_matrix = 0;
	_u_matrix = 0;
	_udot_matrix = 0;

	int j = 0;
	for (const Coordinate& coord : _model.getComponentList<Coordinate>()) {

		if (q_col_map(j) != -1) {
			Array<double> data;
			store.getDataColumn(col_labels[q_col_map(j)], data, _time[0]);
			for (int i = 0; i < _n_frames; ++i) {
				_q_matrix(i, j) = data[i];
			}
		}
		else {
			std::cout << "Coordinate Value: " << coord.getName() << " not found in coordinates_file, assuming 0." << std::endl;
		}

		//GCVSpline q_spline;
		//q_spline.setDegree(5);

        SimmSpline q_spline;
		for (int i = 0; i < _n_frames; ++i) {
			q_spline.addPoint(_time[i], _q_matrix(i, j));
		}

		for (int i = 0; i < _n_frames; ++i) {
			SimTK::Vector x(1);
			x(0) = _time[i];

			std::vector<int> u_order = { 0 };
			_u_matrix(i, j) = q_spline.calcDerivative(u_order, x);

			std::vector<int> udot_order = { 0,0 };
			_udot_matrix(i, j) = q_spline.calcDerivative(udot_order, x);
		}
		j++;
	}

	if (get_print_input_kinematics()) {
		STOFileAdapter sa;
		
		std::vector<double> t;
		for (int i = 0; i < _time.size(); ++i) {
			t.push_back(_time[i]);
		}
		
		std::vector<std::string> labels;
		int i = 0;
		for (auto coord : _model.getComponentList<Coordinate>()) {
			labels.push_back(coord.getName());
		}

		TimeSeriesTable q_table = TimeSeriesTable(t, _q_matrix, labels);
		q_table.addTableMetaData("header", std::string("processed_input_q"));
		q_table.addTableMetaData("inDegrees", std::string("no"));
		q_table.addTableMetaData("nColumns", std::to_string(q_table.getNumColumns() + 1));
		q_table.addTableMetaData("nRows", std::to_string(q_table.getNumRows()));

		sa.write(q_table, get_results_dir() + "/" + get_results_prefix() + "processed_input_q.sto");

		TimeSeriesTable u_table = TimeSeriesTable(t, _u_matrix, labels);
		u_table.addTableMetaData("header", std::string("processed_input_u"));
		u_table.addTableMetaData("inDegrees", std::string("no"));
		u_table.addTableMetaData("nColumns", std::to_string(u_table.getNumColumns() + 1));
		u_table.addTableMetaData("nRows", std::to_string(u_table.getNumRows()));

		sa.write(u_table, get_results_dir() + "/" + get_results_prefix() + "processed_input_u.sto");

		TimeSeriesTable udot_table = TimeSeriesTable(t, _udot_matrix, labels);
		udot_table.addTableMetaData("header", std::string("processed_input_udot"));
		udot_table.addTableMetaData("inDegrees", std::string("no"));
		udot_table.addTableMetaData("nColumns", std::to_string(udot_table.getNumColumns() + 1));
		udot_table.addTableMetaData("nRows", std::to_string(udot_table.getNumRows()));

		sa.write(udot_table, get_results_dir() + "/" + get_results_prefix() + "processed_input_udot.sto");
		
		std::cout << std::endl;
		std::cout << "Printed processed input kinematics to results_dir: " + get_results_dir() << std::endl;
	}
}

void COMAKTool::applyExternalLoads()
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

SimTK::Vector COMAKTool::computeMuscleVolumes() {
    double max_iso_stress = 350000.0; //Muscle Specific Tension

    SimTK::Vector msl_volume(_model.getMuscles().getSize(), 0.0);
    
    int i = 0;
    for (const Muscle& msl : _model.getComponentList<Muscle>()) {
        double l0 = msl.get_optimal_fiber_length();
        double fmax = msl.get_max_isometric_force();

        msl_volume[i] = (l0*fmax) / max_iso_stress;
        i++;
    }
    return msl_volume;
}

void COMAKTool::printCOMAKascii() {
	std::cout <<
		/*
		"\n"
		"#####################################################################################\n"
		"#####################################################################################\n"
		"##                                                                                 ##\n"
		"##      WWWWWWWWW                                                 WWWWWW           ##\n"
		"##   WWWWWWWWWWWWWW                                                WWWWW  WWWWWWW  ##\n"
		"##  WWWWWWWWW    WW     WWWW                             WWWWW      WWWW   WWWW    ##\n"
		"## WWWWWWWWW      W   WWWWWWWWW   WWW WWWWW WWWWW     WWWWWWWWW     WWWW  WWWW     ##\n"
		"## WWWWWWWWWW        WWW   WWWWWW  WWWWWWWWWWWWWWWW  WWWWW  WWWW    WWWW WWWW      ##\n"
		"## WWWWWWWWWWWWWWWW  WWW  WWWWWWW  WWWWWWWWWWWWWWWW  WWWWW   WWWW   WWWWWWWW       ##\n"
		"## WWWWWWWWWWWWWWWW  WWWWWWWWWWWW  WWWWWWWWWWWWWWWW  WWWWWWWWWWWW   WWWWWWWWW      ##\n"
		"##  WWWWWWWWWWWWWWW  WWWWWWWWWWWW  WWWW  WWWN WWWWW  WWWWWWWWWWWW   WWWWW  WWWW    ##\n"
		"##    WWWWWWWWWWW     WWWWWWWWWW   WWW   WWWW  WWW   WWWWWWWW WWW   WWWW    WWWW   ##\n"
		"##      WWWWWW          WWWWW     WWWWW       WWWWW    WWWW  WWWW  WWWWWW  WWWWWWW ##\n"
		"##                                                                                 ##\n"
		"#####################################################################################\n"
		"#####################################################################################\n"
		"##           Concurrent Optimization of Muscle Activations and Kinematics          ##\n"
		"##                                                                                 ##\n"
		"##  Developed by Colin Smith and Darryl Thelen at University of Wisconsin-Madison  ##\n"
		"#####################################################################################\n"
		"#####################################################################################\n\n"*/

"####################################################################################################\n"
"####################################################################################################\n"
"##                        WWWWW                                                                   ##\n"
"##                       WW WW  WWWWWWWWWWWWWWWWWWWWWW         WWW                                ##\n"
"##                       WWW WWWWWWWWWWWWWW     WWWWW  WWWWWWW    WW                              ##\n"
"##                       WW WW  WWWWWWWWWW    WWWWWWWWWWWWWWWWWW   WW                             ##\n"
"##                    WWW   WW     WWWWWW     WWWWWWWWW       WWW  W                              ##\n"
"##                  WWW   WWW   WW  WWWW     WWWWWW        W   W WW                               ##\n"
"##                 WW  WWWWW   W  WW WWW    WWWW WWWWWW    WWWW  WWW                              ##\n"
"##                  WW WWWWW     W WWWWW   W W WW     W    WWWW   WW                              ##\n"
"##                  WWW WWWW     WWW        WWW WWWW       WWWWWW  WWW                            ##\n"
"##                   WWW WWWW        WWWWWWWW            WWWWWW  WWW                              ##\n"
"##                      WWW  WW   WWWW   WWWWWWWW      WWWWWWW WWW                                ##\n"
"##                       WWWW      W WWWWWW        WWWWWW    WWWW                                 ##\n"
"##                            WWWWW    WWWW              WWW                                      ##\n"
"##                           WWWWWW WW     W  WWWWWWWWW                                           ##\n"
"##                           WW WWWWWW     WWWWWWWWWWW                                            ##\n"
"##                          WWWWWWWWWWWWWWWWW WWWWWWWWWWWWWW                                      ##\n"
"##                       WWWWWWWWWWWWWWWWWWWWWWWWWWWWWW WWWWWWWWWWW                               ##\n"
"##                      WWWWWWWW WWWWWWWWWWWWWWW WWWWWWWWWWWWWWWWWWW                              ##\n"
"##                    WWWWWWWWWWWWWWWWWWWWWWWWWWW WW WWWWWWWWWWW WWWW                             ##\n"
"##                    WWWWWWWWW   W    W    WWWWWWWWWWW    WWWW WW WWWW                           ##\n"
"##                 WWWWWWWWWWW   WW   W     WWWWWWW  WWWWWWWWWWWWWWWWWWW                          ##\n"
"##               WWWWWWWWWWWW    WW   W   WWWWWWWWWWWWWWWW WWWWWWWWWWWWWW                         ##\n"
"##             WWWWWW WWWWWWWW   W    W   WWWWWWWWWW        WWWWWWWWWWWWW                         ##\n"
"##             WWWWWWWWWW WWWW      W    WWWWWWWWWWW         WWWWWWWWWWWW                         ##\n"
"##            WW     WWWWWWWWWW    WWW   WWWWWWW WWW         WWWWWWWWWWW                          ##\n"
"##            WW     WWWWWWWWWWWWWW WWWWWWWWWWWWWWWWWWWW    WW   WW WWWW                          ##\n"
"##          WW  WWW WWWWW WWWWWWWWWWWW WWWWWWWWWWWWWWWWWW  WW    WWWWWWW                          ##\n"
"##          WW   W WWWWWW   WWWWWWWWWWWWWWWWWWWWWWWW WWWWW WW  WW  WWWW                           ##\n"
"##            WW   WWWWWW      WWWWWWWWWWWWWWWWWW  WWWWWWW WWWW   WWWWW                           ##\n"
"##              WWWWWWW         WWWWWWWWWWWWWWWW WWWWWWWWW   WWWWWWWWW                            ##\n"
"##                                WWWWWWWWWWWWWWWWWWWW  WW     WWWWW                              ##\n"
"##                                 WWWWWWWWWWWWWWWWW     WWWW                                     ##\n"
"##                                   WW   W    WWWWW    WWWWW                                     ##\n"
"##                                    WWW WW   WWWW     W   WW                                    ##\n"
"##                                     WWWWW   WWW    WWW   WW                                    ##\n"
"##                                     WW WW  WWW  WWWWWWW  WW                                    ##\n"
"##                                     WWW    W   WW        WW                                    ##\n"
"##                                 WWW           WWWW       WW                                    ##\n"
"##                                 WWWWW         WWWWWWW  WWW                                     ##\n"
"##                               WW WW WWWWW     WW WWWWWW                                        ##\n"
"##                               WWWW W  W  W WWWW                                                ##\n"
"##                                   WWWWWWWWW                                                    ##\n"
"##                                                                                                ##\n"
"##                                                        WW                                      ##\n"
"##                                                    WWWWWW                                      ##\n"
"##                                       WWWWWWWW  WWWWWWWWWW                                     ##\n"
"##                                      WWWWWWWWWW WWWWWWWWWWW                                    ##\n"
"##    WWWWWW WWWW      WWWWWWW         WWWWWWWWWWWWWWWWWWWWWWW      WWWWWWWWWW   WWWWWWWW WWWWWWW ##\n"
"##  WWWWWWWWWWWWW   WWWWWWWWWWWWW      WWWWWWWWWWWWWWWW WWWWWWW     WWWWWWWWWW   WWWWWWWW WWWWWWW ##\n"
"## WWWWWW    WWWW  WWWWWW   WWWWWW    WWWWWWW WWWWWWWW  WWWWWWWW    WWWWWWWWW     WWWWWW  WWWW    ##\n"
"## WWWWWW          WWWWWW   WWWWWW    WWWWWWW  WWWWWWW   WWWWWWW    WWW  WWWWW    WWWWWWWWWWW     ##\n"
"## WWWWWW    WWW   WWWWWW   WWWWWW   WWWWWWW   WWWWWWW  WWWWWWWW   WWWWWWWWWWW    WWWWWW WWWWWW   ##\n"
"## WWWWWWW  WWWWW   WWWWWW WWWWWW   WWWWWWWWW WWWWWWWW WWWWWWWWW WWWWW    WWWWWW WWWWWWW  WWWWWWW ##\n"
"##   WWWWWWWWWW       WWWWWWWWW    WWWWWWWWWW WWWWWWWW WWWWWWW   WWWWWW  WWWWWWW WWWWWWWW WWWWWWW ##\n"
"##                                 WWWWWWWWW  WWWWWWWW WWWW                                       ##\n"
"##                                 WWWWWWWWW  WWWWWWW                                             ##\n"
"##                                WWWWWWWW                                                        ##\n"  
"##                                                                                                ##\n"
"####################################################################################################\n"
"####################################################################################################\n"                          
"        ##           Concurrent Optimization of Muscle Activations and Kinematics          ##\n"
"        ##                                                                                 ##\n"
"        ##  Developed by Colin Smith and Darryl Thelen at University of Wisconsin-Madison  ##\n"
"        #####################################################################################\n"
"        #####################################################################################\n\n"


	<< std::endl;

	if (get_verbose() > 0) {
		std::cout << std::endl;
		std::cout << "Prescribed Coordinates:" << std::endl;
		std::cout << "-----------------------" << std::endl;
		for (int i = 0; i < _n_prescribed_coord; ++i) {
			std::cout << _prescribed_coord_name[i] << std::endl;
		}

		std::cout << std::endl;
		std::cout << "Primary Coordinates:" << std::endl;
		std::cout << "--------------------" << std::endl;
		for (int i = 0; i < _n_primary_coord; ++i) {
			std::cout << _primary_coord_name[i] << std::endl;
		}

		std::cout << std::endl;
		std::cout << "Secondary Coordinates:" << std::endl;
		std::cout << "----------------------" << std::endl;
		for (int i = 0; i < _n_secondary_coord; ++i) {
			std::cout << _secondary_coord_name[i] << std::endl;
		}
		std::cout << std::endl;
	}
};

