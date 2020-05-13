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
    constructProperty_secondary_constraint_sim_sweep_time(1.0);
    constructProperty_secondary_coupled_coordinate_start_value(0.0);
    constructProperty_secondary_coupled_coordinate_stop_value(0.0);
    constructProperty_secondary_constraint_sim_integrator_accuracy(1e-6);
    constructProperty_secondary_constraint_sim_internal_step_limit(-1);
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
    //Make results directory
    int makeDir_out = IO::makeDir(get_results_directory());
    if (errno == ENOENT && makeDir_out == -1) {
        OPENSIM_THROW(Exception, "Could not create " +
            get_results_directory() +
            "Possible reason: This tool cannot make new folder with subfolder.");
    }

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
        get_secondary_constraint_sim_sweep_time() << std::endl;

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
    
    //Initialize Model
    Model model = _model;
    model.setUseVisualizer(get_use_visualizer());
    model.initSystem();

    for (Muscle& msl : model.updComponentList<Muscle>()) {
        if (msl.getConcreteClassName() == "Millard2012EquilibriumMuscle") {
            msl.set_ignore_activation_dynamics(true);
            msl.set_ignore_tendon_compliance(true);
        }
    }
        
    for(auto& cc_const : model.updComponentList<CoordinateCouplerConstraint>()){
        std::string cc_coord_name = cc_const.getDependentCoordinateName();
        Coordinate& coord = model.updCoordinateSet().get(cc_coord_name);
        coord.set_locked(false);
    }

    //Set coordinate types
    for (auto& coord : model.updComponentList<Coordinate>()) {
        if (getProperty_secondary_coordinates().findIndex(coord.getAbsolutePathString()) > -1) {
            coord.set_locked(false);
            coord.set_clamped(false);
        }
        else if (coord.getAbsolutePathString() == get_secondary_coupled_coordinate()){
            coord.set_locked(false);
            coord.set_clamped(false);
            coord.set_prescribed(true);
        }
        else {
            coord.set_locked(true);
        }
    }

    Coordinate& coupled_coord = model.updComponent<Coordinate>(get_secondary_coupled_coordinate());
    
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

    //Initialize Simulation 
    //---------------------

    if (get_use_visualizer()) {
        SimTK::Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
        viz.setBackgroundColor(SimTK::White);
        viz.setShowSimTime(true);
    }

    StatesTrajectory result_states;



    //Perform settling simulation
    //---------------------------
    double settle_start = 0.0;
    double settle_stop = get_secondary_constraint_sim_settle_time();

    //prescribe coupled coord
    Constant settle_func = Constant(start_value);
    coupled_coord.set_prescribed_function(settle_func);

    SimTK::State state = model.initSystem();

    //prescribe muscle force
    for (Muscle& msl : model.updComponentList<Muscle>()) {
        msl.overrideActuation(state, true);
        double value = msl.getMaxIsometricForce()*0.01;
        msl.setOverrideActuation(state, value);
    }
    model.equilibrateMuscles(state);

    //setup integrator
    SimTK::CPodesIntegrator integrator(model.getSystem(), SimTK::CPodes::BDF, SimTK::CPodes::Newton);
    integrator.setAccuracy(get_secondary_constraint_sim_integrator_accuracy());
    if (get_secondary_constraint_sim_internal_step_limit() != -1) {
        integrator.setInternalStepLimit(
            get_secondary_constraint_sim_internal_step_limit());
    }
    SimTK::TimeStepper timestepper(model.getSystem(), integrator);

    timestepper.initialize(state);
    
    double dt = 0.01;

    int nSteps = round((settle_stop - settle_start) / dt);

    for (int i = 0; i <= nSteps; ++i) {
        timestepper.stepTo(i*dt);
        state = timestepper.getState();

        result_states.append(state);

        if (get_verbose() > 0) {
            std::cout << state.getTime() << std::endl;
        }
    }

    SimTK::Vector settled_secondary_values(_secondary_coord_path.getSize());

    //Save secondardy coord values to initialize sweep simulation
    
    for (int c = 0; c < _secondary_coord_path.getSize(); c++) {
        std::string secondary_coord = _secondary_coord_path[c];
        Coordinate& coord = model.updComponent<Coordinate>(secondary_coord);
        settled_secondary_values.set(c, coord.getValue(state));       
    }


    //Perform Sweep Simulation
    //------------------------

    //setup quadratic sweep function
    double Vx = get_secondary_constraint_sim_settle_time();
    double Vy = start_value;
    double Px = Vx + get_secondary_constraint_sim_sweep_time();
    double Py = stop_value;
    double a = (Py - Vy) / SimTK::square(Px - Vx);

    double C1 = a;
    double C2 = -2 * a * Vx;
    double C3 = a * SimTK::square(Vx) + Vy;

    SimTK::Vector coefficients(3);
    coefficients.set(0, C1);
    coefficients.set(1, C2);
    coefficients.set(2, C3);

    PolynomialFunction sweep_func = PolynomialFunction(coefficients);
    coupled_coord.set_prescribed_function(sweep_func);

    state = model.initSystem();

    //prescribe muscle force
    for (Muscle& msl : model.updComponentList<Muscle>()) {
        msl.overrideActuation(state, true);
        double value = msl.getMaxIsometricForce()*0.01;
        msl.setOverrideActuation(state, value);
    }
    model.equilibrateMuscles(state);

    //set settled secondary coordinate values
    for (int c = 0; c < _secondary_coord_path.getSize(); c++) {
        std::string secondary_coord = _secondary_coord_path[c];
        Coordinate& coord = model.updComponent<Coordinate>(secondary_coord);
        coord.setValue(state, settled_secondary_values(c));
    }

    double sweep_start = settle_stop + dt;
    double sweep_stop = Px;

    nSteps = round((sweep_stop - sweep_start) / dt);

    //Setup storage for computing constraint functions
    TimeSeriesTable q_table;
    SimTK::RowVector q_row(model.getNumCoordinates());
    std::vector<std::string> q_names;

    for (Coordinate& coord : _model.updComponentList<Coordinate>()) {
        q_names.push_back(coord.getAbsolutePathString() + "/value");
    }

    q_table.setColumnLabels(q_names);

    //setup integrator
    SimTK::CPodesIntegrator sweep_integrator(model.getSystem(), SimTK::CPodes::BDF, SimTK::CPodes::Newton);
    sweep_integrator.setAccuracy(get_secondary_constraint_sim_integrator_accuracy());
    if (get_secondary_constraint_sim_internal_step_limit() != -1) {
        sweep_integrator.setInternalStepLimit(
            get_secondary_constraint_sim_internal_step_limit());
    }
    SimTK::TimeStepper sweep_timestepper(model.getSystem(), sweep_integrator);

    sweep_timestepper.initialize(state);

    for (int i = 0; i <= nSteps; ++i) {

        sweep_timestepper.stepTo(sweep_start + i*dt);
        state = sweep_timestepper.getState();

        result_states.append(state);

        int j = 0;
        for (const auto& coord : model.getComponentList<Coordinate>()) {
            q_row(j) = coord.getValue(state);
            j++;
        }
        q_table.appendRow(state.getTime(), q_row);

        if (get_verbose() > 0) {
            std::cout << state.getTime() << std::endl;
        }
    }



    //Compute Coupled Constraint Functions
    std::vector<double> time = q_table.getIndependentColumn();

    SimTK::Vector ind_data = q_table.getDependentColumn(get_secondary_coupled_coordinate() + "/value");

    SimTK::Matrix data(time.size(), _n_secondary_coord);

    for (int j = 0; j < _n_secondary_coord; ++j) {
        std::string path = _secondary_coord_path[j];
        SimTK::Vector col_data = q_table.getDependentColumn(path + "/value");
        
        for (int i = 0; i < nSteps; ++i) {
            data(i, j) = col_data(i);
            
        }
    }

    double ind_max = SimTK::max(ind_data);
    double ind_min = SimTK::min(ind_data);
    
    int npts = 15;
    double step = (ind_max - ind_min) / npts;
    
    SimTK::Vector ind_pt_data(npts);

    for (int i = 0; i < npts; ++i) {
        ind_pt_data(i) = ind_min + i * step;
    }

    _secondary_constraint_functions.clearAndDestroy();

    for (int j=0; j < _n_secondary_coord; ++j){
        std::string path = _secondary_coord_path[j];
        
        SimTK::Vector secondary_data = data(j);

        //GCVSpline data_fit = GCVSpline(5, data.size(), &ind_data[0], &data[0]);
        SimmSpline data_fit = SimmSpline(secondary_data.size(), &ind_data[0], &secondary_data[0]);

        
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


