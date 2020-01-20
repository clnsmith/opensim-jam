/* -------------------------------------------------------------------------- *
 *                                 COMAKTool.cpp                              *
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
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim.h>
#include "COMAKTool.h"
#include "COMAKTarget.h"
#include "HelperFunctions.h"
#include "Smith2018ArticularContactForce.h"


using namespace OpenSim;
using namespace SimTK;

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
    _directoryOfSetupFile = "";
}

COMAKTool::COMAKTool(const std::string file) : Object(file) {
    constructProperties();
    updateFromXMLDocument();
    _directoryOfSetupFile = IO::getParentDirectory(file);
    IO::chDir(_directoryOfSetupFile);
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void COMAKTool::constructProperties()
{
    constructProperty_model_file("");
    constructProperty_ik_motion_file("");
    constructProperty_external_loads_file("");
    constructProperty_results_dir("");
    constructProperty_results_prefix("");
    constructProperty_use_visualizer(false);
    constructProperty_print_input_kinematics(false);
    constructProperty_verbose(0);
    constructProperty_start_time(-1);
    constructProperty_stop_time(-1);
    constructProperty_time_step(-1);
    constructProperty_lowpass_filter_frequency(-1);

    constructProperty_prescribed_coordinates();
    constructProperty_primary_coordinates();
    constructProperty_COMAKSecondaryCoordinateSet(COMAKSecondaryCoordinateSet());
    constructProperty_replace_force_set(false);
    constructProperty_force_set_file("");
    constructProperty_COMAKCostFunctionParameterSet(COMAKCostFunctionParameterSet());

    constructProperty_max_iterations(50);
    constructProperty_max_change_rotation(0.05);
    constructProperty_max_change_translation(0.005);
    constructProperty_udot_tolerance(1.0);
    constructProperty_udot_worse_case_tolerance(50.0);
    constructProperty_unit_udot_epsilon(0.000005);
    constructProperty_contact_energy_weight(0.0);

    constructProperty_equilibriate_secondary_coordinates_at_start(true);
    constructProperty_settle_threshold(0.00001);
    constructProperty_settle_accuracy(0.00001);
    constructProperty_settle_sim_results_prefix("");
    constructProperty_settle_sim_results_dir("");
    constructProperty_print_settle_sim_results(false);

    constructProperty_AnalysisSet(AnalysisSet());
}

void COMAKTool::setModel(Model& model) {
    _model = model;
}

void COMAKTool::run()
{
    printCOMAKascii();
    initialize();
    performCOMAK();
}

void COMAKTool::initialize()
{
    setModel(Model(get_model_file()));
    updateModelForces();

    _model.initSystem();

    // Verfiy Coordinate Properties
    for (Coordinate& coord : _model.updComponentList<Coordinate>()) {
        std::string name = coord.getName();
        std::string path = coord.getAbsolutePathString();
        bool isSecondary = false;

        // Reset coordinates listed in settings file to full path
        int ind = getProperty_prescribed_coordinates().findIndex(name);
        if (ind > -1) {
            set_prescribed_coordinates(ind, path);
        }

        ind = getProperty_primary_coordinates().findIndex(name);
        if (ind > -1) {
            set_primary_coordinates(ind, path);
        }

        for (int i = 0; i < get_COMAKSecondaryCoordinateSet().getSize(); ++i) {
            COMAKSecondaryCoordinate secondary_coord = get_COMAKSecondaryCoordinateSet().get(i);
            if (secondary_coord.get_coordinate() == name || secondary_coord.get_coordinate() == path) {
                secondary_coord.set_coordinate(path);
                isSecondary = true;
            }
        }

        
        bool isPrescribed = (getProperty_prescribed_coordinates().findIndex(path) > -1);
        bool isPrimary = (getProperty_primary_coordinates().findIndex(path) > -1);
        
        // Set all unlisted coordinates to prescribed
        if (!isPrescribed && !isPrimary && !isSecondary) {
            append_prescribed_coordinates(path);

            std::cout << "WARNING: Coordinate (" << name <<
                ") was not listed in COMAKTool params file. Assumed PRESCRIBED."
                << std::endl;
        }

        // Make sure coordinate is not listed multiple times
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

    // Make sure Coordinates exist in model and no duplicates    
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

    for (int i = 0; i < get_COMAKSecondaryCoordinateSet().getSize(); ++i) {
        std::string path = get_COMAKSecondaryCoordinateSet().get(i).get_coordinate();

        std::cout << path << std::endl;

        try { _model.getComponent<Coordinate>(path); }
        catch (Exception){
            OPENSIM_THROW(Exception,"secondary_coordinate: " + path + "not found in model.")
        }

        int n = 0;
        for (int j = 0; j < get_COMAKSecondaryCoordinateSet().getSize(); ++j) {
            if (path == get_COMAKSecondaryCoordinateSet().get(j).get_coordinate()) n++;
        }
        OPENSIM_THROW_IF(n>1, Exception, path + "listed multiple times in secondary_coordinates")    
    }



    // Count number of each coordinate type
    _n_prescribed_coord = getProperty_prescribed_coordinates().size();
    _n_primary_coord = getProperty_primary_coordinates().size();
    _n_secondary_coord = get_COMAKSecondaryCoordinateSet().getSize();

    _prescribed_coord_name.setSize(_n_prescribed_coord);
    _prescribed_coord_path.setSize(_n_prescribed_coord);
    _prescribed_coord_index.setSize(_n_prescribed_coord);

    _primary_coord_name.setSize(_n_primary_coord);
    _primary_coord_path.setSize(_n_primary_coord);
    _primary_coord_index.setSize(_n_primary_coord);

    _secondary_coord_name.setSize(_n_secondary_coord);
    _secondary_coord_path.setSize(_n_secondary_coord);
    _secondary_coord_index.setSize(_n_secondary_coord);


    // Organize coordinates in nice stuctures
    for (int i = 0; i < _n_prescribed_coord; ++i) {
        _prescribed_coord_path[i] = get_prescribed_coordinates(i);
        _prescribed_coord_name[i] = _model.getComponent<Coordinate>(get_prescribed_coordinates(i)).getName();
    }

    for (int i = 0; i < _n_primary_coord; ++i) {
        _primary_coord_path[i] = get_primary_coordinates(i);
        _primary_coord_name[i] = _model.getComponent<Coordinate>(get_primary_coordinates(i)).getName();
    }

    for (int i = 0; i < _n_secondary_coord; ++i) {
        _secondary_coord_path[i] = get_COMAKSecondaryCoordinateSet().get(i).get_coordinate();
        _secondary_coord_name[i] = _model.getComponent<Coordinate>(_secondary_coord_path[i]).getName();
    }

    // Find the index of each coordinate in the updComponentList<Coordinate>
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

    //Organize COMAKSecondaryCoordinate Properties
    _secondary_coord_damping.resize(_n_secondary_coord);
    _secondary_coord_damping = 0;

    for (int i = 0; i < _n_secondary_coord; ++i) {
        for (int j = 0; j < get_COMAKSecondaryCoordinateSet().getSize(); ++j) {
            COMAKSecondaryCoordinate sec_coord = get_COMAKSecondaryCoordinateSet().get(j);
            if (sec_coord.get_coordinate() == _secondary_coord_path[i]) {
                _secondary_coord_damping[i] = sec_coord.get_comak_damping();                
            }
        }
    }

    //Compute Optimal Forces For Actuators
    _n_actuators = 0;
    _n_muscles = 0;
    _n_non_muscle_actuators = 0;

    for (ScalarActuator &actuator : _model.updComponentList<ScalarActuator>()) {
        _n_actuators++;
    }
    _optimal_force.resize(_n_actuators);

    int i = 0;
    for (const Muscle& msl : _model.updComponentList<Muscle>()) {
        //_optimal_force[i] = msl.getMaxIsometricForce();
        _optimal_force[i] = msl.getMaxIsometricForce()*cos(msl.getPennationAngleAtOptimalFiberLength()*SimTK::Pi/180);
        i++;
        _n_muscles++;
        _muscle_path.append(msl.getAbsolutePathString());
    }

    for (ScalarActuator &actuator : _model.updComponentList<ScalarActuator>()) {
        if (Object::isObjectTypeDerivedFrom< Muscle >(actuator.getConcreteClassName())) {
            continue;
        }
        _optimal_force[i] = actuator.getOptimalForce();
        i++;
        _n_non_muscle_actuators++;
        _non_muscle_actuator_path.append(actuator.getAbsolutePathString());
    }

    _muscle_volumes = computeMuscleVolumes();

    if (get_verbose() > 0) {
        int w = 15;
        std::cout << "\nMuscle Properties" << std::endl;
        std::cout << std::setw(15) << "name " << std::setw(15) << "Fmax" << 
            std::setw(15) << "Volume" << std::endl;
        i = 0;
        for (const Muscle& msl : _model.getComponentList<Muscle>()) {
            double l0 = msl.get_optimal_fiber_length();
            double fmax = msl.get_max_isometric_force();

            std::cout << std::setw(15) << msl.getName() <<  std::setw(15) << 
                fmax << std::setw(15) << _muscle_volumes[i] << std::endl;
            i++;
        }
    }

    // Setup optimization parameters
    // parameters vector ordered
    // muscle activations 
    // non muscle activations
    // secondary coord values

    _optim_parameters.resize(_n_actuators + _n_secondary_coord);
    _optim_parameters = 0;

    _n_parameters = _optim_parameters.size();
    _prev_parameters = _optim_parameters;

    _optim_parameter_names.setSize(_n_parameters);

    int p = 0;
    for (Muscle &msl : _model.updComponentList<Muscle>()) {
        _optim_parameter_names[p] = msl.getName();
        p++;
    }
    for (ScalarActuator &actuator : _model.updComponentList<CoordinateActuator>()) {
        if (Object::isObjectTypeDerivedFrom< Muscle >(actuator.getConcreteClassName())) {
            continue;
        }
        _optim_parameter_names[p] = actuator.getName();
        p++;
    }
    for (int p = 0; p < _n_secondary_coord; ++p) {
        _optim_parameter_names[_n_actuators + p] = _secondary_coord_name[p];
    }

    //Organize COMAKCostFunctionParameters
    for (Muscle &msl : _model.updComponentList<Muscle>()) {
        bool found_msl = false;
        for (int j = 0; j < get_COMAKCostFunctionParameterSet().getSize(); ++j) {
            COMAKCostFunctionParameter parameter = get_COMAKCostFunctionParameterSet().get(j);
            if (parameter.get_actuator() == msl.getAbsolutePathString()) {
                found_msl = true;
                _cost_muscle_weights.cloneAndAppend(parameter.get_weight());
            }
        }
        if(found_msl == false){
            _cost_muscle_weights.cloneAndAppend(Constant(1.0));
            
        }
    }


    //Add in reserve actuators for computing COMAK damping gradients
    for (int i = 0; i < _n_secondary_coord; ++i) {
        const Coordinate& coord = _model.updComponent<Coordinate>(
            _secondary_coord_path[i]);

        //Check for illegal named actuators in model
        std::string damping_actuator_name = coord.getName() + "__COMAK_DAMPING__";
        
        if (_model.findComponent<CoordinateActuator>(ComponentPath(damping_actuator_name))
            != nullptr) {
            OPENSIM_THROW(Exception, damping_actuator_name + 
                " is an illegal name for CoordinateActuator. ");
        }

        CoordinateActuator* res_act = new CoordinateActuator();
        res_act->setName(damping_actuator_name);
        res_act->setOptimalForce(0);
        res_act->setMaxControl(1);
        res_act->setMinControl(-1);
        res_act->set_coordinate(coord.getName());
        _model.addComponent(res_act);

        _secondary_damping_actuator_path.append(res_act->getAbsolutePathString());
    }
   
    //Add Analysis set
    AnalysisSet aSet = get_AnalysisSet();
    int size = aSet.getSize();

    for(int i=0;i<size;i++) {
        Analysis *analysis = aSet.get(i).clone();
        _model.addAnalysis(analysis);
    }


}

void COMAKTool::performCOMAK()
{
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

    if (get_use_visualizer()) {
        _model.setUseVisualizer(true);
    }
    state = _model.initSystem();

    //Setup Results Storage
    StatesTrajectory states_trajectory;
    AnalysisSet& analysisSet = _model.updAnalysisSet();

    std::vector<std::string> actuator_names;
    
    for (int m = 0; m < _n_muscles; ++m) {
        actuator_names.push_back(_muscle_path[m]);
    }
    for (int m = 0; m < _n_non_muscle_actuators; ++m) {
        actuator_names.push_back(_non_muscle_actuator_path[m]);
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
    
    // initialize optimization parameters
    for (int i = 0; i < _n_muscles; ++i) {
        _optim_parameters[i] = 0.02;
    }
    for (int i = 0; i < _n_secondary_coord; ++i) {
        _optim_parameters[i + _n_actuators] = init_secondary_values(i);
    }

    //Set initial Secondary Qs
    _dt = _time[1] - _time[0];

    _prev_secondary_value.resize(_n_secondary_coord);

    for (int j = 0; j < _n_secondary_coord; ++j) {
        Coordinate& coord = _model.updComponent<Coordinate>(_secondary_coord_path[j]);
        coord.setValue(state, init_secondary_values(j), false);		
        _prev_secondary_value(j) = init_secondary_values(j);
    }

    //Visualizer Settings
    SimTK::Visualizer* viz;
    if (get_use_visualizer()) {
        viz = &_model.updVisualizer().updSimbodyVisualizer();
        viz->setBackgroundColor(SimTK::White);
        viz->setShowSimTime(true);
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

        //Iterate for COMAK Solution		
        double max_udot_error = SimTK::Infinity;
        SimTK::Vector iter_max_udot_error(get_max_iterations(),0.0);
        std::vector<std::string> iter_max_udot_coord(get_max_iterations(), "");
        SimTK::Matrix iter_parameters(get_max_iterations(), _n_parameters,0.0);

        int iter;
        for (iter = 0; iter < get_max_iterations(); ++iter) {

            std::cout << std::endl;
            std::cout << "--------------------------------------------------------------------------------" << std::endl;
            std::cout << "Frame: " << frame_num << "\t"
                      << "Time: " << _time[i] << "\t"
                      << "Iteration: " << iter << std::endl;
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
                _optim_parameters[j] = 0;
                coord_act.setOverrideActuation(state,0);		
                j++;
            }*/

            ComakTarget target = ComakTarget(state, &_model, ~_udot_matrix[i],
                _optim_parameters, _optim_parameter_names,
                _primary_coord_path, _secondary_coord_path,
                _muscle_path, _non_muscle_actuator_path, 
                _secondary_damping_actuator_path, false);

            SimTK::Vector msl_weight(_n_muscles);
            for (int m = 0; m < _n_muscles; ++ m) {
                msl_weight(m) = _cost_muscle_weights.get(m).calcValue(SimTK::Vector(1, _time[i]));
            }

            target.setCostFunctionWeight(msl_weight);
            target.setMaxChangeRotation(get_max_change_rotation());
            target.setMaxChangeTranslation(get_max_change_translation());
            target.setUdotTolerance(get_udot_tolerance());
            target.setUnitUdotEpsilon(get_unit_udot_epsilon());
            target.setDT(_dt);
            target.setOptimalForces(_optimal_force);
            target.setPrevSecondaryValues(_prev_secondary_value);
            target.setMuscleVolumes(_muscle_volumes);
            target.setSecondaryCoordinateDamping(_secondary_coord_damping);
            target.setContactEnergyWeight(get_contact_energy_weight());
            target.initialize();

            SimTK::OptimizerAlgorithm algorithm = SimTK::InteriorPoint;
            SimTK::Optimizer optimizer(target, algorithm);

            optimizer.setDiagnosticsLevel(0);
            optimizer.setMaxIterations(500);
            optimizer.setConvergenceTolerance(0.00000001);
            
            optimizer.useNumericalGradient(false);
            optimizer.useNumericalJacobian(false);

            if (algorithm == SimTK::InteriorPoint) {
                // Some IPOPT-specific settings
                optimizer.setAdvancedBoolOption("warm_start", true);
                //optimizer.setAdvancedStrOption("expect_infeasible_problem", "yes");
                optimizer.setAdvancedRealOption("obj_scaling_factor", 1);
                optimizer.setAdvancedRealOption("nlp_scaling_max_gradient", 1);
            }

            for (int m = 0; m < 10; ++m) {
                try {
                    optimizer.optimize(_optim_parameters);
                    break;
                }
                catch (SimTK::Exception::Base ex) {
                    std::cout << "COMAK Optimization failed, upping the parameter bounds: " << ex.getMessage() << std::endl;
                    target.setParameterBounds(m);
                    optimizer.setOptimizerSystem(target);
                }

            }
            iter_parameters[iter] = ~_optim_parameters;            
                        
            setStateFromComakParameters(state, _optim_parameters);

            _model.realizeAcceleration(state);

            //Output Optimization Results
            if (get_verbose() > 0) {
                printOptimizationResultsToConsole(_optim_parameters);
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
                _optim_parameters = ~iter_parameters[min_iter];
                std::cout << "Using best iteration (" << min_iter << ") with max udot error: " << min_val << std::endl;
            }
            else {
                _optim_parameters = _prev_parameters;
                std::cout << "No iteration has max udot error less than worst case tolerance (" << get_udot_worse_case_tolerance() << ").\n"
                    << "Resetting optimization parameters to previous time step solution." << std::endl;
            }
            setStateFromComakParameters(state, _optim_parameters);

            //Save data about failed convergence
            _bad_frames.push_back(frame_num);
            _bad_times.push_back(_time[i]);
            _bad_udot_errors.push_back(min_val);
            _bad_udot_coord.push_back(bad_coord);
        }
        

        //Store Solution
        _model.realizeAcceleration(state);
        _prev_parameters = _optim_parameters;

        for (int m = 0; m < _n_secondary_coord; ++m) {
            Coordinate& coord = _model.updComponent<Coordinate>(_secondary_coord_path[m]);
            _prev_secondary_value(m) = coord.getValue(state);
        }

        //Save the results
        //Record parameters
        if (i == 0) {
            analysisSet.begin(state);
        }
        else {
            analysisSet.step(state, i);
        }

        states_trajectory.append(state);

        SimTK::RowVector activations(_n_actuators);
        SimTK::RowVector forces(_n_actuators);

        for (int m = 0; m < _n_actuators; ++m) {
            activations(m) = _optim_parameters(m);
            forces(m) = _optim_parameters(m)*_optimal_force(m);
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

        //Visualize the Results
        if (get_use_visualizer()) {
            int k = 0;
            for (int m = 0; m < _n_muscles; ++m) {
                Muscle &msl = _model.updComponent<Muscle>(_muscle_path[m]);
                msl.overrideActuation(state, false);
                _model.realizePosition(state);
                msl.setActivation(state, _optim_parameters[k]);
                k++;
            }

            _model.realizeAcceleration(state);
            viz->drawFrameNow(state);

            for (int m = 0; m < _n_muscles; ++m) {
                Muscle &msl = _model.updComponent<Muscle>(_muscle_path[m]);
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
    states_table.addTableMetaData("header", std::string("ModelStates"));
    states_table.addTableMetaData("nRows", std::to_string(states_table.getNumRows()));
    states_table.addTableMetaData("nColumns", std::to_string(states_table.getNumColumns()+1));
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

void COMAKTool::setStateFromComakParameters(SimTK::State& state, const SimTK::Vector& parameters) {
    //Set Muscle Activations to Optimized
    int j = 0;
    for (int m = 0; m < _n_muscles; ++m) {
        Muscle &msl = _model.updComponent<Muscle>(_muscle_path[m]);
        msl.overrideActuation(state, true);
        double force = _optimal_force[j] * parameters[j];
        msl.setOverrideActuation(state,force);
        j++;
    }
    //Set Reserve Activations to Optimized
    for (int m = 0; m < _n_non_muscle_actuators; ++m) {
        ScalarActuator &actuator = _model.updComponent<ScalarActuator>(_non_muscle_actuator_path[m]);
        actuator.overrideActuation(state, true);
        double force = _optimal_force[j] * parameters[j];
        actuator.setOverrideActuation(state,force);
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

SimTK::Vector COMAKTool::equilibriateSecondaryCoordinates() 
{
    Model settle_model = _model;
    SimTK::State state = settle_model.initSystem();

    std::cout << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
    std::cout << "Performing forward simulation to equilibriate secondary kinematics" << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;

    for (Muscle& msl : settle_model.updComponentList<Muscle>()) {
        if (msl.getConcreteClassName() == "Millard2012EquilibriumMuscle") {
            msl.set_ignore_activation_dynamics(true);
            msl.set_ignore_tendon_compliance(true);
        }
    }

    if (get_use_visualizer()) {
        settle_model.setUseVisualizer(true);
    }

    state = settle_model.initSystem();
    
    if (get_use_visualizer()) {
        SimTK::Visualizer& viz = settle_model.updVisualizer().updSimbodyVisualizer();
        viz.setBackgroundColor(SimTK::White);
        viz.setShowSimTime(true);
    }

    // Set initial pose and Lock all coordinates except secondary 
    int nCoord = 0;
    for (Coordinate& coord : settle_model.updComponentList<Coordinate>()) {
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

    // Don't lock coordinates that are constrained by 
    // CoordinateCouplerConstraint
    for(auto& cc_const : settle_model.updComponentList<CoordinateCouplerConstraint>()){
        std::string cc_coord_name = cc_const.getDependentCoordinateName();
        Coordinate& coord = settle_model.updCoordinateSet().get(cc_coord_name);
        coord.setLocked(state,false);
    }

    settle_model.assemble(state);
    settle_model.realizeVelocity(state);

    // Prescribe Muscle Force
    for (Muscle& msl : settle_model.updComponentList<Muscle>()) {
        msl.overrideActuation(state, true);
        double value = msl.getMaxIsometricForce()*0.02;
        msl.setOverrideActuation(state, value);
    }

    StatesTrajectory result_states;

    // Store Secondary Coordinate Values (to check if simulation is settled)
    SimTK::Vector prev_sec_coord_value(_n_secondary_coord);

    for (int k = 0; k < _n_secondary_coord; k++) {
        Coordinate& coord = settle_model.updComponent<Coordinate>(_secondary_coord_path[k]);
        prev_sec_coord_value(k) = coord.getValue(state);
    }

    // Perform settling simulation
    SimTK::CPodesIntegrator integrator(settle_model.getSystem(),
        SimTK::CPodes::BDF, SimTK::CPodes::Newton);

    integrator.setAccuracy(get_settle_accuracy());

    SimTK::TimeStepper timestepper(settle_model.getSystem(), integrator);
    timestepper.initialize(state);
    
    double dt = 0.01;

    double max_coord_delta = SimTK::Infinity;
    int i = 1;
    while (max_coord_delta > get_settle_threshold()){
        timestepper.stepTo(i*dt);
        state = timestepper.getState();
        result_states.append(state);
        
        if (get_verbose() > 0) {
            std::cout << std::endl;
            std::cout << "Time: " << state.getTime() << std::endl;
            std::cout << "\t\t VALUE \t\tDELTA" << std::endl;
        }

        //Compute Delta Coordinate
        max_coord_delta = 0;
        for (int k = 0; k < _n_secondary_coord; k++) {
            Coordinate& coord = settle_model.updComponent<Coordinate>(_secondary_coord_path[k]);
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
        TimeSeriesTable states_table = result_states.exportToTable(settle_model);
        states_table.addTableMetaData("header", std::string("COMAK Settle Simulation States"));
        states_table.addTableMetaData("nRows", std::to_string(states_table.getNumRows()));
        states_table.addTableMetaData("nColumns", std::to_string(states_table.getNumColumns()+1));
        states_table.addTableMetaData("inDegrees", std::string("no"));

        STOFileAdapter sto;
        IO::makeDir(get_settle_sim_results_dir());
        std::string basefile = get_settle_sim_results_dir() + "/" + get_settle_sim_results_prefix();
        sto.write(states_table, basefile + "_states.sto");
    }

    if (get_verbose() > 0) {
        std::cout << "Settled secondary_coordinate values" << std::endl;
        std::cout << "-----------------------------------" << std::endl;

        for (int i = 0; i < _n_secondary_coord; ++i) {
            Coordinate& coord = settle_model.updComponent<Coordinate>(
                _secondary_coord_path[i]);

            std::cout << coord.getName() << ": " <<
                coord.getValue(state) << std::endl;
        }
    }

    //Collect Settled Secondary Q values;
    SimTK::Vector secondary_q(_n_secondary_coord);
    for (int i = 0; i < _n_secondary_coord; ++i) {
        Coordinate& coord = settle_model.updComponent<Coordinate>(
            _secondary_coord_path[i]);
        
        secondary_q(i) = coord.getValue(state);

        std::cout << coord.getName() << ": " << 
            coord.getValue(state) << std::endl;
    }
    return secondary_q;
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

void COMAKTool::updateModelForces()
{
    // If replacing force set read in from model file, clear it here
    if (get_replace_force_set()){
        // Can no longer just remove the model's forces.
        // If the model is connected, then the model will
        // maintain a list of subcomponents that refer to garbage.
        _model.cleanup();
        _model.updForceSet().setSize(0);
    }

    // Load force set(s)
    if (!get_force_set_file().empty()) {
        std::cout << "Adding force object set from " <<
            get_force_set_file() << std::endl;

        ForceSet *forceSet = new ForceSet(get_force_set_file(), true);
        _model.updForceSet().append(*forceSet);
    }
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
"####################################################################################################\n" 
"####################################################################################################\n"
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
"####################################################################################################\n"
"####################################################################################################\n"                          
"        ##           Concurrent Optimization of Muscle Activations and Kinematics          ##\n"
"        ##                                                                                 ##\n"
"        ##               Developed by Colin Smith [1,2] and Darryl Thelen [1]              ##\n"
"        ##                       1. University of Wisconsin-Madison                        ##\n"
"        ##                                   2. ETH Zurich                                 ##\n"
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

void COMAKTool::printOptimizationResultsToConsole(const SimTK::Vector& parameters) {
    int w = 20;

    std::cout << std::left << "\nOptimized Muscles:" << std::endl;
    std::cout << std::setw(w) << "name" << std::setw(w) << "activation" << std::setw(w) << "force" << std::endl;
    int p = 0;
    for (int k = 0; k < _n_muscles; ++k) {
        std::cout << std::setw(w) << _optim_parameter_names[p] << std::setw(w) << parameters[p] << std::setw(w) << parameters[p] * _optimal_force[p] << std::endl;
        p++;
    }

    std::cout << "\nOptimized Non Muscle Actuators:" << std::endl;
    std::cout << std::setw(w) << "name" << std::setw(w) << "activation" << std::setw(w) << "force" << std::endl;
    for (int k = 0; k < _n_non_muscle_actuators; ++k) {
        std::cout << std::setw(w) << _optim_parameter_names[p] << std::setw(w) << parameters[p] << std::setw(w) << parameters[p] * _optimal_force[p] << std::endl;
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
    //target.printPerformance(parameters);

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

COMAKSecondaryCoordinate::COMAKSecondaryCoordinate()
{
    constructProperties();
}

void COMAKSecondaryCoordinate::constructProperties() 
{
    constructProperty_coordinate("");
    constructProperty_comak_damping(1.0);
    constructProperty_max_change(0.005);
}

COMAKSecondaryCoordinateSet::COMAKSecondaryCoordinateSet()
{
    constructProperties();
}

void COMAKSecondaryCoordinateSet::constructProperties() 
{
   
}

COMAKCostFunctionParameter::COMAKCostFunctionParameter()
{
    constructProperties();
}

void COMAKCostFunctionParameter::constructProperties() 
{
    constructProperty_actuator("");
    constructProperty_weight(Constant(1.0));
    constructProperty_desired_activation(Constant(0.0));
}

COMAKCostFunctionParameterSet::COMAKCostFunctionParameterSet()
{
    constructProperties();
}

void COMAKCostFunctionParameterSet::constructProperties() 
{
   
}