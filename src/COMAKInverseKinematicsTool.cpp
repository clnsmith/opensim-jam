/* -------------------------------------------------------------------------- *
 *                     COMAKInverseKinematicsTool.cpp                         *
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
#include "COMAKInverseKinematicsTool.h"
#include "HelperFunctions.h"


using namespace OpenSim;
using namespace SimTK;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================

//_____________________________________________________________________________
/**
 * Default constructor.
 */
COMAKInverseKinematicsTool::COMAKInverseKinematicsTool() 
{
    constructProperties();
}

COMAKInverseKinematicsTool::COMAKInverseKinematicsTool(const std::string file) : Object(file) {
    constructProperties();
    updateFromXMLDocument();
        
    _directoryOfSetupFile = IO::getParentDirectory(file);	
    IO::chDir(_directoryOfSetupFile); 
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void COMAKInverseKinematicsTool::constructProperties()
{
    constructProperty_model_file("");
    constructProperty_results_directory("");
    constructProperty_results_prefix("");

    constructProperty_perform_secondary_constraint_sim(true);
    constructProperty_secondary_coordinates();
    constructProperty_secondary_coupled_coordinate("");
    constructProperty_secondary_constraint_sim_settle_time(1.0);
    constructProperty_secondary_constraint_sim_time(1.0);
    constructProperty_secondary_coupled_coordinate_start_value(0.0);
    constructProperty_secondary_coupled_coordinate_stop_value(0.0);
    constructProperty_secondary_constraint_sim_integrator_accuracy(1e-6);
    constructProperty_secondary_constraint_function_file(
        "secondary_coordinate_constraint_functions.xml");
    constructProperty_print_secondary_constraint_sim_results(false);

    constructProperty_perform_inverse_kinematics(true);
    constructProperty_constrained_model_file("");
    constructProperty_InverseKinematicsTool(InverseKinematicsTool());
    constructProperty_use_visualizer(false);
    constructProperty_verbose(0);
}

void COMAKInverseKinematicsTool::initialize()
{
    IO::makeDir(get_results_directory());

    _model = Model(get_model_file());

    std::string function_file = get_secondary_constraint_function_file();

    if (get_secondary_constraint_function_file() == "") {
        OPENSIM_THROW(Exception, "secondary_constraint_function file not set.")
    }
    
    _model.initSystem();

    //Verfiy Coordinate Properties
    
    for (Coordinate& coord : _model.updComponentList<Coordinate>()) {
        std::string name = coord.getName();
        std::string path = coord.getAbsolutePathString();

        //Reset to full path
        if (get_secondary_coupled_coordinate() == name) {
            set_secondary_coupled_coordinate(path);
        }

        int ind = getProperty_secondary_coordinates().findIndex(name);

        if (ind > -1) {
            set_secondary_coordinates(ind, path);
        }
    }

    //Make sure Coordinate exists in model and no duplicates	
    std::string name = get_secondary_coupled_coordinate();
    try { _model.getComponent<Coordinate>(name); }
    catch (Exception) {
        OPENSIM_THROW(Exception, "secondary_coupled_coord: " + name + " not found in model.")
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
    _n_secondary_coord = getProperty_secondary_coordinates().size();

    _secondary_coord_name.setSize(_n_secondary_coord);
    _secondary_coord_path.setSize(_n_secondary_coord);
    _secondary_coord_index.setSize(_n_secondary_coord);


    for (int i = 0; i < _n_secondary_coord; ++i) {
        _secondary_coord_path[i] = get_secondary_coordinates(i);
        _secondary_coord_name[i] = _model.getComponent<Coordinate>(get_secondary_coordinates(i)).getName();
    }

    int nCoord = 0;
    for (Coordinate& coord : _model.updComponentList<Coordinate>()) {
        std::string path = coord.getAbsolutePathString();

        int ind = _secondary_coord_path.findIndex(path);
        if (ind > -1) {
            _secondary_coord_index[ind] = nCoord;
        }
        nCoord++;
    }

    std::cout << std::endl;
    std::cout << "Secondary Coordinates:" << std::endl;
    std::cout << "----------------------" << std::endl;
    for (int i = 0; i < _n_secondary_coord; ++i) {
        std::cout << _secondary_coord_name[i] << std::endl;
    }
    std::cout << std::endl;
    std::cout << "Secondary Coupled Coordinate: " <<
        get_secondary_coupled_coordinate() << std::endl;

    std::cout << "Settle Time: " << 
        get_secondary_constraint_sim_settle_time()<< std::endl;

    std::cout << "Sweep Time: " <<
        get_secondary_constraint_sim_time() << std::endl;

    std::cout << "Sweep secondary_coupled_coordinate start value: " <<
        get_secondary_coupled_coordinate_start_value() << std::endl;

    std::cout << "Sweep secondary_coupled_coordinate stop value: " <<
        get_secondary_coupled_coordinate_stop_value() << std::endl;

    std::cout << std::endl;


    _state = _model.initSystem();
}

void COMAKInverseKinematicsTool::run()
{
    //Secondary Constraint Simulation
    if (get_perform_secondary_constraint_sim()) {
        performIKSecondaryConstraintSimulation();
    }

    //Inverse Kinematics 
    if (get_perform_inverse_kinematics()) {
        performIK();
    }
}


void COMAKInverseKinematicsTool::performIKSecondaryConstraintSimulation() {
    std::cout << "Performing IK Secondary Constraint Simulation..." << std::endl;
    
    Model model = _model;
    model.initSystem();

    //Setup Prescribed Function
    double start_time = get_secondary_constraint_sim_settle_time();
    double stop_time = start_time + get_secondary_constraint_sim_time();
    double t[] = { 0, start_time, stop_time };

    double start_value;
    double stop_value;

    if (model.getComponent<Coordinate>(get_secondary_coupled_coordinate()).getMotionType() == Coordinate::MotionType::Rotational) {
        start_value = get_secondary_coupled_coordinate_start_value() * SimTK::Pi / 180;
        stop_value = get_secondary_coupled_coordinate_stop_value() * SimTK::Pi / 180;
    }
    else {
        start_value = get_secondary_coupled_coordinate_start_value();
        stop_value = get_secondary_coupled_coordinate_stop_value();
    }
    
    double x[] = { start_value, start_value, stop_value };
    PiecewiseLinearFunction func(3, t, x);

    //Set coordinate Types
    for (auto& coord : model.updComponentList<Coordinate>()) {
        if (getProperty_secondary_coordinates().findIndex(coord.getAbsolutePathString()) > -1) {
            coord.set_locked(false);
        }
        else if (coord.getAbsolutePathString() == get_secondary_coupled_coordinate()){
            coord.set_locked(false);
            coord.set_prescribed(true);
            coord.set_prescribed_function(func);
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

    for (Muscle& msl : model.updComponentList<Muscle>()) {
        if (msl.getConcreteClassName() == "Millard2012EquilibriumMuscle") {
            msl.set_ignore_activation_dynamics(true);
            msl.set_ignore_tendon_compliance(true);
        }
    }

    //Initialize Model
    model.setUseVisualizer(get_use_visualizer());
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

    //Setup Results Storage
    TimeSeriesTable q_table;
    SimTK::RowVector q_row(model.getNumCoordinates());
    std::vector<std::string> q_names;

    for (Coordinate& coord : _model.updComponentList<Coordinate>()) {
        q_names.push_back(coord.getAbsolutePathString() + "/value");

    }

    q_table.setColumnLabels(q_names);
    StatesTrajectory result_states;

    SimTK::CPodesIntegrator integrator(model.getSystem(), SimTK::CPodes::BDF, SimTK::CPodes::Newton);
    integrator.setAccuracy(get_secondary_constraint_sim_integrator_accuracy());
    
    SimTK::TimeStepper timestepper(model.getSystem(), integrator);

    timestepper.initialize(state);

    double dt = 0.01;

    int nSteps = round((finalTime - initialTime) / dt);

    for (int i = 0; i <= nSteps; ++i) {
        model.realizeReport(state);

        timestepper.stepTo(i*dt);
        state = timestepper.getState();

        result_states.append(state);

        int j = 0;
        for (const auto& coord : model.getComponentList<Coordinate>()) {
            q_row(j) = coord.getValue(state);
            j++;
        }
        q_table.appendRow(state.getTime(), q_row);
    }

    //Compute Coupled Constraint Functions
    std::vector<double> time = q_table.getIndependentColumn();
    int start_frame = round((start_time - initialTime) / dt);
    int nCutFrames = nSteps - start_frame;

    SimTK::Vector ind_data = q_table.getDependentColumn(get_secondary_coupled_coordinate() + "/value");
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

    _secondary_constraint_functions.clearAndDestroy();

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
        
        _secondary_constraint_functions.adoptAndAppend(spline);
    }

    //Print Secondardy Constraint Functions to file
    _secondary_constraint_functions.print(get_secondary_constraint_function_file());

    //Write Outputs
    if (get_print_secondary_constraint_sim_results()) {
        std::cout << "Printing secondary constraint simulation results: " <<
            get_results_directory() << std::endl;

         std::string name = "secondary_constraint_sim_states";

        TimeSeriesTable states_table = result_states.exportToTable(model);
        states_table.addTableMetaData("header", name);
        states_table.addTableMetaData("nRows", std::to_string(states_table.getNumRows()));
        states_table.addTableMetaData("nColumns", std::to_string(states_table.getNumColumns()+1));

        std::string states_file =
            get_results_directory() + "/secondary_constraint_sim_states.sto";
        STOFileAdapter sto_file_adapt;
        sto_file_adapt.write(states_table, states_file);
    }
}

void COMAKInverseKinematicsTool::performIK() 
{
    Model model = _model;
    model.initSystem();

    try {
        _secondary_constraint_functions = 
            FunctionSet(get_secondary_constraint_function_file());
    }
    catch (Exception){
        OPENSIM_THROW(Exception,"Function file: " + 
            get_secondary_constraint_function_file() + " does not exist.");

    }

    for (int i = 0; i < getProperty_secondary_coordinates().size(); ++i) {
        std::string name = get_secondary_coordinates(i);
        std::string coord_name = model.getComponent<Coordinate>(name).getName();
        std::string ind_coord_name = model.getComponent<Coordinate>(get_secondary_coupled_coordinate()).getName();

        CoordinateCouplerConstraint* cc_constraint = new CoordinateCouplerConstraint();

        cc_constraint->setIndependentCoordinateNames(Array<std::string>(ind_coord_name, 1, 2));
        cc_constraint->setDependentCoordinateName(coord_name);
        cc_constraint->setFunction(_secondary_constraint_functions.get(name));
        cc_constraint->setName(coord_name + "_function");

        model.addComponent(cc_constraint);
    }
    if (!get_constrained_model_file().empty()) {
        model.print(get_constrained_model_file());
    }

    //SimTK::State state = model.initSystem();

    /*if (get_use_visualizer()) {
        SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
        viz.setBackgroundColor(SimTK::White);
        viz.setShowSimTime(true);
    }*/
    upd_InverseKinematicsTool().setModel(model);

    upd_InverseKinematicsTool().run();
}


