/* -------------------------------------------------------------------------- *
 *                               COMAKTarget.cpp                              *
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
#include "ComakTarget.h"
#include "Smith2018ArticularContactForce.h"
#include <OpenSim.h>
using namespace OpenSim;


//==============================================================================
// CONSTRUCTOR
//==============================================================================

ComakTarget::
ComakTarget(SimTK::State s,Model* aModel, const SimTK::Vector& observed_udot,
    const SimTK::Vector& init_parameters, 
    const Array<std::string>& parameter_names,
    Array<std::string> primary_coords, Array<std::string> secondary_coords,
    Array<std::string> muscle_path,
    Array<std::string> non_muscle_actuator_path,
    Array<std::string> secondary_damping_actuator_path,
    bool useMusclePhysiology)

{
    //Set Verbosity
    _verbose = 0;
    
    // Set Parameters
    _primary_coords = primary_coords;
    _nPrimaryCoord = primary_coords.size();
    _secondary_coords = secondary_coords;
    _nSecondaryCoord = secondary_coords.size();

    _muscle_path = muscle_path;
    _non_muscle_actuator_path = non_muscle_actuator_path;

    _secondary_damping_actuator_path
        = secondary_damping_actuator_path;
    _useMusclePhysiology = useMusclePhysiology;
    _state = s;
    _observed_udot = observed_udot;
    _init_parameters = init_parameters;
    _parameter_names = parameter_names;
    _activationExponent = 2.0;
    _model = aModel;
}

void ComakTarget::initialize(){
    //Number of Parameters
    _nMuscles = _muscle_path.size();
    _nNonMuscleActuators = _non_muscle_actuator_path.size();
    _nActuators = _nMuscles + _nNonMuscleActuators;

    _nParameters = _nActuators + _nSecondaryCoord;
    setNumParameters(_nParameters);
    setParameterBounds(1);
    
    //Number of Constraints
    int nC = 0;
    int nCoord = 0;
    for (const Coordinate& coord : _model->getComponentList<Coordinate>()) {
        if (_primary_coords.findIndex(coord.getAbsolutePathString()) > -1) {
            _constraint_names.append(coord.getName());
            nC++;
        }
        else if(_secondary_coords.findIndex(coord.getAbsolutePathString())> -1) {
            _constraint_names.append(coord.getName());
            nC++;
        }
        nCoord++;
    }
    _nConstraints = nC;
    _nCoordinates = nCoord;

    setNumEqualityConstraints(nC);
    setNumLinearEqualityConstraints(nC);
    setNumInequalityConstraints(0);

    //Initial Secondary Coordinates
    _init_secondary_values.resize(_nSecondaryCoord);

    for (int i = 0; i < _nSecondaryCoord; ++i) {
        _init_secondary_values[i] = _init_parameters[_nActuators + i];
    }
    

    if(_muscle_weight.size() == 0){
        _muscle_weight.resize(_nMuscles);
        _muscle_weight = 1.0;
    }
    if(_desired_act.size() == 0){
        _desired_act.resize(_nMuscles);
        _desired_act = 0.0;
    }
    
    if(_muscle_volumes.size() == 0){
        _muscle_volumes.resize(_nMuscles);
        _muscle_volumes = 1.0;
    }
    
   

    //Precompute Constraint Matrix
    precomputeConstraintMatrix();
    if (false) {
    //if (_verbose > 0) {
        std::cout << std::endl;
        std::cout << "Num Parameters: " << _nParameters << std::endl;
        std::cout << "Num Constraints: " << _nConstraints << std::endl;
        std::cout << "Num Actuators: " << _nActuators << std::endl;
        std::cout << "Num Coordinates: " << _nCoordinates << std::endl;
        std::cout << "Num Secondary Coordinates: " << _secondary_coords.size() << std::endl;
        std::cout << "Num Primary Coordinates: " << _primary_coords.size() << std::endl;
        std::cout << std::endl;
    }
}

void ComakTarget::precomputeConstraintMatrix() {
    _constraint_initial_udot.resize(_nConstraints);
    _constraint_desired_udot.resize(_nConstraints);

    SimTK::Vector sim_udot(_nCoordinates);
    

    computeSimulatedAcceleration(_state, _init_parameters, sim_udot);
    _initial_udot = sim_udot;

    int i = 0;
    int j = 0;
    for (const Coordinate& coord : _model->getComponentList<Coordinate>()) {
        if (_primary_coords.findIndex(coord.getAbsolutePathString()) > -1) {
            _constraint_desired_udot[i] = _observed_udot[j];
            _constraint_initial_udot[i] = _initial_udot[j];
            i++;
        }
        if (_secondary_coords.findIndex(coord.getAbsolutePathString()) > -1) {
            _constraint_desired_udot[i] = 0;
            _constraint_initial_udot[i] = _initial_udot[j];
            i++;
        }
        j++;
    }

    //constraint matrix
    computeUnitUdot(_state, _init_parameters);
}

//==============================================================================
// OPTIMIZATION FUNCTIONS
//==============================================================================
//------------------------------------------------------------------------------
// PERFORMANCE
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Compute performance given parameters.
 *
 * @param parameters Vector of optimization parameters.
 * @param performance Value of the performance criterion.
 * @return Status (normal termination = 0, error < 0).
 */
int ComakTarget::
objectiveFunc(const SimTK::Vector &parameters, const bool new_parameters, SimTK::Real &performance) const
{
    double msl_cost = 0.0;
    int p = 0;

    //Muscle Activations
    for (int i = 0; i < _nMuscles; i++) {
        msl_cost += 1000000.0 *_muscle_volumes(i)*_muscle_weight(i)*pow(fabs(parameters(p)-_desired_act(i)), _activationExponent);
        p++;
    }
    
    //Non Muscle Actuators
    double non_muscle_cost = 0.0;
    for (int i = 0; i < _nNonMuscleActuators; i++) {
        non_muscle_cost += 1000.0 * pow(fabs(parameters(p)*_optimalForce[p]), _activationExponent);
        p++;
    }

    double contact_cost = 0.0;
    for (int i = 0; i < _nSecondaryCoord; ++i) {
        double dq = parameters(p) - _prev_secondary_values(i);
        contact_cost += _contact_energy_weight * dq * _secondary_coord_unit_energy[i];
        p++;
    }

    performance = msl_cost + non_muscle_cost + contact_cost;

    if (_verbose > 1) {
        std::cout << "Total Cost: " << performance << "\t";
        std::cout << "Muscle Cost: " << msl_cost << "\t";
        std::cout << "Non Muscle Actuator Cost: " << non_muscle_cost << "\t";
        std::cout << "Contact Cost: " << contact_cost << std::endl;
    }
    return(0);
}
//______________________________________________________________________________
/**
 * Compute the gradient of performance given parameters.
 *
 * @param parameters Vector of optimization parameters.
 * @param gradient Derivatives of performance with respect to the parameters.
 * @return Status (normal termination = 0, error < 0).
 */
int ComakTarget::
gradientFunc(const SimTK::Vector &parameters, const bool new_parameters, SimTK::Vector &gradient) const
{
    gradient = 0;

    int p = 0;
    for (int i = 0; i < _nMuscles; i++) {
        if (parameters[p]-_desired_act(i) < 0) {
            gradient[p] += -1.0 * _activationExponent * 1000000.0 *_muscle_volumes(i) * _muscle_weight(i) * pow(fabs(parameters[p]-_desired_act(i)), _activationExponent - 1.0);
        }
        else {
            gradient[p] += _activationExponent * 1000000.0 *_muscle_volumes(i) * _muscle_weight(i) * pow(fabs(parameters[p] - _desired_act(i)), _activationExponent - 1.0);
        }
        p++;
    }

    for (int i = 0; i < _nNonMuscleActuators; i++) {
        if (parameters[p] < 0) {
            gradient[p] += -1.0 * _activationExponent * 1000.0 * pow(fabs(parameters[p]*_optimalForce[p]), _activationExponent - 1.0);
        }
        else {
            gradient[p] += _activationExponent * 1000.0 * pow(fabs(parameters[p]*_optimalForce[p]), _activationExponent - 1.0);
        }
        p++;
    }

    for (int i = 0; i < _nSecondaryCoord; ++i) {
        gradient[p] += _contact_energy_weight * _secondary_coord_unit_energy[i];
        p++;
    }

    if (_verbose > 1) {
        std::cout << "Cost Gradient" << std::endl;
        for (int i = 0; i < _nParameters; ++i) {
            std::cout << _parameter_names[i] << " " << gradient[i] << std::endl;
        }
    }
    return(0);
}

//------------------------------------------------------------------------------
// CONSTRAINT
//------------------------------------------------------------------------------
//______________________________________________________________________________
/**
 * Compute acceleration constraints given parameters.
 *
 * @param parameters Vector of optimization parameters.
 * @param constraints Vector of optimization constraints.
 * @return Status (normal termination = 0, error < 0).
 */
int ComakTarget::
constraintFunc(const SimTK::Vector &parameters, bool new_parameters, SimTK::Vector &constraints) const
{
    SimTK::Vector udots(_nConstraints,0.0);
    for (int i = 0; i < _nConstraints; ++i) {
        double udot = _constraint_initial_udot[i];

        //Muscles
        int p = 0;
        for (int j = 0; j < _nMuscles; ++j) {
            double msl_force = (parameters[p] - _init_parameters[p]) * _optimalForce[p];
            udot += msl_force * _msl_unit_udot(i, j);
            p++;
        }

        //Non Muscle Actuators
        for (int j = 0; j < _nNonMuscleActuators; ++j) {
            double non_muscle_actuator_force = (parameters[p] - _init_parameters[p]) * _optimalForce[p];
            udot += non_muscle_actuator_force * _non_muscle_actuator_unit_udot(i, j);
            p++;
        }

        //Secondary Coordinates
        SimTK::Vector secondary_u(_nSecondaryCoord,0.0);

        for (int j = 0; j < _nSecondaryCoord; ++j) {
            double dq = parameters[p] - _init_secondary_values[j];
            udot += dq * _secondary_coord_unit_udot(i,j);
            secondary_u(j) = dq / _dt;
            p++;
        }

        //COMAK Damping to penalize changes in secondary coords
        for (int j = 0; j < _nSecondaryCoord; ++j) {
           udot += secondary_u(j) * _secondary_damping_unit_udot(i, j);
        }

        constraints[i] = udot - _constraint_desired_udot[i];
        udots[i] = udot;
    }
    return(0);
}


//______________________________________________________________________________
/**
 * Compute the gradient of constraint given parameters.
 *
 * @param parameters Vector of parameters.
 * @param jac Derivative of constraint with respect to the parameters.
 * @return Status (normal termination = 0, error < 0).
 */
int ComakTarget::constraintJacobian(const SimTK::Vector &parameters, bool new_parameters, SimTK::Matrix &jac) const
{
    //Constraints = Rows
    //Parameters = Columns

    jac = 0;
    for (int i = 0; i < _nConstraints; ++i) {
        
        //Muscles
        int p = 0;
        for (int j = 0; j < _nMuscles; ++j) {
            jac(i,p) = _optimalForce[p] * _msl_unit_udot(i, j);
            p++;
        }

        //Non Muscle Actuators
        for (int j = 0; j < _nNonMuscleActuators; ++j) {
            jac(i,p) = _optimalForce[p] * _non_muscle_actuator_unit_udot(i, j);
            p++;
        }

        //Secondary Coordinates

        for (int j = 0; j < _nSecondaryCoord; ++j) {
            jac(i,p) = _secondary_coord_unit_udot(i,j);
            jac(i,p) += _secondary_damping_unit_udot(i,j)/_dt;
            p++;
        }
    }
    return 0;
}
//=============================================================================
// ACCELERATION
//=============================================================================
//
void ComakTarget::computeSimulatedAcceleration(SimTK::State s, const SimTK::Vector &parameters, SimTK::Vector &sim_udot) 
{
    //Apply Muscle Forces
    int j = 0;
    for (int i = 0; i < _nMuscles; ++i) {
        Muscle &msl = _model->updComponent<Muscle>(_muscle_path[i]);
        msl.overrideActuation(s, true);
        double force = _optimalForce[j] * parameters[j];
        msl.setOverrideActuation(s,force);
        j++;
    }

    //Apply Non Muscle Actuator Forces
    for (int i = 0; i < _nNonMuscleActuators; ++i) {
        ScalarActuator &actuator = _model->updComponent<ScalarActuator>(_non_muscle_actuator_path[i]);
        actuator.overrideActuation(s, true);
        double force = _optimalForce[j] * parameters[j];
        actuator.setOverrideActuation(s,force);
        j++;
    }

    //Set Secondary Coordinates 
    for (int i = 0; i < _nSecondaryCoord; ++i) {
        double value = parameters(_nActuators + i);
        _model->updComponent<Coordinate>(_secondary_coords[i]).setValue(s, value, false);
    }
    
    //Set ALL speeds to zero
    /*for (Coordinate& coord : _model->updComponentList<Coordinate>()) {
        double speed = 0;
        coord.setSpeedValue(s, speed);
    }*/

    _model->assemble(s);
    _model->realizeAcceleration(s);

    int i = 0;
    for (const Coordinate& coord : _model->updComponentList<Coordinate>()) {
        sim_udot(i) = coord.getAccelerationValue(s);
        i++;
    }
}

void ComakTarget::computeUnitUdot(SimTK::State s, const SimTK::Vector& parameters) 
/**
*
* act_unit_udot: rows - coordinates, columns - actuators
*/
{
    _msl_unit_udot.resize(_nConstraints, _nMuscles);
    _non_muscle_actuator_unit_udot.resize(_nConstraints, _nNonMuscleActuators);
    _secondary_coord_unit_udot.resize(_nConstraints, _nSecondaryCoord);
    _secondary_damping_unit_udot.resize(_nConstraints, _nSecondaryCoord);
    _secondary_coord_unit_energy.resize(_nSecondaryCoord);
       
    _msl_unit_udot = -1;
    _non_muscle_actuator_unit_udot = -1;
    _secondary_coord_unit_udot = -1;
    _secondary_damping_unit_udot = -1;
    _secondary_coord_unit_energy = -1;

    //Apply Muscle Forces
    int j = 0;
    for (int i = 0; i < _nMuscles; ++i) {
        Muscle &msl = _model->updComponent<Muscle>(_muscle_path[i]);
        msl.overrideActuation(s, true);
        double force = _optimalForce[j] * parameters[j];
        msl.setOverrideActuation(s,force);
        j++;
    }

    //Apply Non Muscle Actuator Forces
    for (int i = 0; i < _nNonMuscleActuators; ++i) {
        ScalarActuator &actuator = _model->updComponent<ScalarActuator>(_non_muscle_actuator_path[i]);
        actuator.overrideActuation(s, true);
        double force = _optimalForce[j] * parameters[j];
        actuator.setOverrideActuation(s,force);
        j++;
    }

    //Set Secondary Kinematics to Current
    for (int i = 0; i < _nSecondaryCoord; ++i) {
        double value = parameters(_nActuators + i);
        _model->updComponent<Coordinate>(_secondary_coords[i]).setValue(s, value, false);
    }
    //Set ALL speeds to zero
    /*for (Coordinate& coord : _model->updComponentList<Coordinate>()) {
        double speed = 0;
        coord.setSpeedValue(s, speed);
    }*/
    _model->assemble(s);
    _model->realizeAcceleration(s);

    //Compute Current Contact Energy
    double current_cnt_energy = 0;
    
    for (Smith2018ArticularContactForce& cnt_frc : _model->updComponentList<Smith2018ArticularContactForce>()) {
        current_cnt_energy += cnt_frc.getOutputValue<double>(s,"potential_energy");
    }

    //Compute Muscle Unit Udot
    j = 0;
    for (int i = 0; i < _nMuscles; ++i) {
        Muscle &msl = _model->updComponent<Muscle>(_muscle_path[i]);

        double force = parameters[j] * _optimalForce[j];

        msl.setOverrideActuation(s, force + 1.0);

        _model->realizeAcceleration(s);
        
        int k = 0;
        for (const Coordinate& coord : _model->updComponentList<Coordinate>()) {
            if (_primary_coords.findIndex(coord.getAbsolutePathString()) > -1 ||
                _secondary_coords.findIndex(coord.getAbsolutePathString()) > -1) {

                _msl_unit_udot(k, j) = coord.getAccelerationValue(s) - _constraint_initial_udot(k);
                k++;
            }
        }
        msl.setOverrideActuation(s, force);

        j++;
    }

    //Compute Non Muscle Actuator Unit Udot
    j = 0;
    for (int i = 0; i < _nNonMuscleActuators; ++i) {
        ScalarActuator &actuator = _model->updComponent<ScalarActuator>(_non_muscle_actuator_path[i]);

        double force = parameters[_nMuscles + j] * _optimalForce[_nMuscles + j];
        actuator.setOverrideActuation(s, force + 1.0);

        _model->realizeAcceleration(s);
        
        int k = 0;
        for (const Coordinate& coord : _model->updComponentList<Coordinate>()) {
            if (_primary_coords.findIndex(coord.getAbsolutePathString()) > -1 ||
                _secondary_coords.findIndex(coord.getAbsolutePathString()) > -1) {

                _non_muscle_actuator_unit_udot(k, j) = (coord.getAccelerationValue(s) - _constraint_initial_udot(k));
                k++;
            }
        }
        actuator.setOverrideActuation(s, force);

        j++;
    }

    //Compute Secondary Coordinate Unit Udot
    for (int j = 0; j < _nSecondaryCoord; ++j) {		
        double value = parameters(_nActuators + j) + _unit_udot_epsilon;

        _model->updComponent<Coordinate>(_secondary_coords[j]).setValue(s, value,true);
        
        _model->realizeAcceleration(s);
        
        int k = 0;
        for (const Coordinate& coord : _model->updComponentList<Coordinate>()) {
            if (_primary_coords.findIndex(coord.getAbsolutePathString()) > -1 ||
                _secondary_coords.findIndex(coord.getAbsolutePathString()) > -1) {

                _secondary_coord_unit_udot(k, j) = (coord.getAccelerationValue(s) - _constraint_initial_udot(k)) / _unit_udot_epsilon;
                k++;
            }
        }
        
        //Contact Energy dot
        double cnt_energy = 0;
        for (Smith2018ArticularContactForce& cnt_frc : _model->updComponentList<Smith2018ArticularContactForce>()) {
            cnt_energy += cnt_frc.getOutputValue<double>(s,"potential_energy"); 
        }
        _secondary_coord_unit_energy(j) = (cnt_energy - current_cnt_energy)/_unit_udot_epsilon;

        value = parameters(_nActuators + j);
        _model->updComponent<Coordinate>(_secondary_coords[j]).setValue(s, value, true);
    }
    _model->realizeAcceleration(s);

    //Compute COMAK damping unit udot
    SimTK::Vector org_reserve_frc(_nSecondaryCoord,0.0);

    /*for (int j = 0; j < _nSecondaryCoord; ++j) {
        std::string coord_name = _model->updComponent<Coordinate>(_secondary_coords[j]).getName();
        //org_reserve_frc(j) = _model->updComponent<CoordinateActuator>("/forceset/" + coord_name + "_reserve").getActuation(s);
        org_reserve_frc(j) = _model->updComponent<CoordinateActuator>(_secondary_damping_actuator_path[j]).getActuation(s);
    }*/

    for (int j = 0; j < _nSecondaryCoord; ++j) {
        std::string coord_name = _model->updComponent<Coordinate>(_secondary_coords[j]).getName();

        CoordinateActuator& actuator = _model->updComponent<CoordinateActuator>(_secondary_damping_actuator_path[j]);

        actuator.setOverrideActuation(s, 1.0);

        _model->realizeAcceleration(s);

        
        int k = 0;
        for (const Coordinate& coord : _model->updComponentList<Coordinate>()) {
            if (_primary_coords.findIndex(coord.getAbsolutePathString()) > -1 ||
                _secondary_coords.findIndex(coord.getAbsolutePathString()) > -1) {
                double damping_force_udot = coord.getAccelerationValue(s) - _constraint_initial_udot(k);

                _secondary_damping_unit_udot(k, j) = -damping_force_udot * _secondary_coord_damping[j];
                k++;
            }
        }

        actuator.setOverrideActuation(s, 0.0);		
    }

    _model->realizeAcceleration(s);
}

void ComakTarget::setParameterBounds(double scale) {

    SimTK::Vector lower_bounds(_nParameters), upper_bounds(_nParameters);
    int p = 0;
    for (int i = 0; i < _nMuscles; ++i) {
        Muscle &msl = _model->updComponent<Muscle>(_muscle_path[i]);

        double min_value = msl.getMinControl();
        double max_value = msl.getMaxControl();

        lower_bounds(p) = min_value;
        upper_bounds(p) = max_value;
        p++;
    }

    for (int i = 0; i < _nNonMuscleActuators; ++i) {
        ScalarActuator &actuator = _model->updComponent<ScalarActuator>(_non_muscle_actuator_path[i]);

        double min_value = actuator.getMinControl()*scale;
        double max_value = actuator.getMaxControl()*scale;

        lower_bounds(p) = min_value;
        upper_bounds(p) = max_value;
        p++;
    }

    for (int j = 0; j < _nSecondaryCoord; j++) {
        Coordinate& coord = _model->updComponent<Coordinate>(_secondary_coords[j]);
            lower_bounds(p) = _init_parameters[p] - _max_change[j] * scale;
            upper_bounds(p) = _init_parameters[p] + _max_change[j] * scale;
        p++;
    }
    setParameterLimits(lower_bounds, upper_bounds);

    if (_verbose > 0) {
        std::cout << "Parameter Limits: \t initial value \t lower bound \t upper bound" << std::endl;
        std::cout << "-----------------" << std::endl;

        for (int i = 0; i < _nParameters; ++i) {
            std::cout << i << " " << _parameter_names[i] << ":\t" << _init_parameters[i] << "\t" << lower_bounds(i) << "\t" << upper_bounds(i) << std::endl;
        }
    }
}

void ComakTarget::printPerformance(SimTK::Vector parameters) {
    bool notNeeded = false;
    SimTK::Real performance;
    objectiveFunc(parameters, notNeeded, performance);

    std::cout << "\nOptimization Cost Function: " << performance << std::endl;
}

