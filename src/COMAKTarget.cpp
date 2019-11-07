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
#include "Blankevoort1991Ligament.h"
#include <OpenSim.h>
using namespace OpenSim;


//==============================================================================
// CONSTRUCTOR
//==============================================================================

ComakTarget::
ComakTarget(SimTK::State s, Model* aModel, const SimTK::Vector& observed_udot,
	const SimTK::Vector& init_parameters,
	Array<std::string> primary_coords, Array<std::string> secondary_coords,
	bool useMusclePhysiology)

{
	//Set Verbosity
	_verbose = 0;
	
	// Set Parameters
	_primary_coords = primary_coords;
	_nPrimaryCoord = primary_coords.size();
	_secondary_coords = secondary_coords;
	_nSecondaryCoord = secondary_coords.size();

	_useMusclePhysiology = useMusclePhysiology;
	_state = s;
	_observed_udot = observed_udot;
	_init_parameters = init_parameters;
	_activationExponent = 2.0;
	_model = aModel;
}
void ComakTarget::initialize(){
	//Number of Parameters
	int nA = 0;
	for (ScalarActuator &actuator : _model->updComponentList<ScalarActuator>()) {
		nA++;
	}
	_nActuators = nA;
	_nParameters = _nActuators + _nSecondaryCoord;
	setNumParameters(_nParameters);

    _nMuscles = _model->getMuscles().getSize();

    _nReserveActuators = 0;
    for (auto coord_act : _model->getComponentList<CoordinateActuator>()) {
        _nReserveActuators++;
    }
     

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

   
	
    // Store Parameter Names
	_parameter_names.setSize(_nParameters);

	int p = 0;
	for (Muscle &msl : _model->updComponentList<Muscle>()) {
		_parameter_names[p] = msl.getName();
		p++;
	}
    for (CoordinateActuator &reserve : _model->updComponentList<CoordinateActuator>()) {
		_parameter_names[p] = reserve.getName();
		p++;
	}
	for (int j = 0; j < _nSecondaryCoord; j++) {
		Coordinate& coord = _model->updComponent<Coordinate>(_secondary_coords[j]);
		_parameter_names[p] = coord.getName();
        p++;
	}

     setParameterBounds(1);

	//Initial Secondary Coordinates
	_init_secondary_values.resize(_nSecondaryCoord);

	for (int i = 0; i < _nSecondaryCoord; ++i) {
		_init_secondary_values[i] = _init_parameters[_nActuators + i];
	}
	

	_weight.resize(_nActuators);
	_desired_act.resize(_nActuators);
    _msl_volumes.resize(_nMuscles);

	_weight = 1.0;
	_desired_act = 0.0;

	//Precompute Constraint Matrix
	precomputeConstraintMatrix();

	if (_verbose > 0) {
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
		msl_cost += 1000000.0 *_msl_volumes(i)*_weight(i)*pow(fabs(parameters(p)-_desired_act(i)), _activationExponent);
        p++;
	}
	
    //Reserve Actuators
    double reserve_cost = 0.0;
    for (int i = 0; i < _nReserveActuators; i++) {
        reserve_cost += 1000.0 * pow(fabs(parameters(p)*_optimalForce[p]), _activationExponent);
        p++;
    }

    double contact_cost = 0.0;
    for (int i = 0; i < _nSecondaryCoord; ++i) {
        double dq = parameters(p) - _prev_secondary_values(i);
        contact_cost += _contact_energy_weight * dq * _secondary_coord_unit_energy[i];
        p++;
    }
    	
	//Activations
	/*for (int i = 0; i < _nActuators; i++) {
		p += pow(fabs(parameters(i)), _activationExponent);
	}*/
    

	performance = msl_cost + reserve_cost + contact_cost;
    

	//if (_verbose > 1) {
	if(false){
        std::cout << "Total Cost: " << performance << "\t";
        std::cout << "Muscle Cost: " << msl_cost << "\t";
        std::cout << "Reserve Cost: " << reserve_cost << "\t";
        std::cout << "Contact Cost: " << contact_cost << std::endl;
		/*std::cout << "Parameter Values: \n";
		for (int i = 0; i < _nParameters; ++i) {
			std::cout << _parameter_names[i] << " " << parameters[i] << std::endl;
		}*/
		//std::cout << "\n" << std::endl;
	}
    //std::cin.ignore();

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
    /*
    for (int i = 0; i < _nActuators; i++) {
		if (parameters[i]-_desired_act(i) < 0) {
			gradient[i] += -1.0 * _activationExponent * pow(fabs(parameters[i]), _activationExponent - 1.0);
		}
		else {
			gradient[i] += _activationExponent * pow(fabs(parameters[i]), _activationExponent - 1.0);
		}
	}*/
    int p = 0;
	for (int i = 0; i < _nMuscles; i++) {
		if (parameters[p]-_desired_act(i) < 0) {
			gradient[p] += -1.0 * _activationExponent * 1000000.0 *_msl_volumes(i) * _weight(i) * pow(fabs(parameters[p]-_desired_act(i)), _activationExponent - 1.0);
        }
		else {
			gradient[p] += _activationExponent * 1000000.0 *_msl_volumes(i) * _weight(i) * pow(fabs(parameters[p] - _desired_act(i)), _activationExponent - 1.0);
        }
        p++;
	}

    for (int i = 0; i < _nReserveActuators; i++) {
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

    if (false) {
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

        //Reserve Actuators
        for (int j = 0; j < _nReserveActuators; ++j) {
            double reserve_force = (parameters[p] - _init_parameters[p]) * _optimalForce[p];
            //double reserve_force = parameters[p] * _optimalForce[p];
            udot += reserve_force * _reserve_unit_udot(i, j);
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
        
    if (false) {
        std::cout << "\n\nConstraint Parameters" << std::endl;
        for (int i = 0; i < _nParameters; ++i) {
            std::cout << _parameter_names[i] << " " << parameters[i] << std::endl;
        }
        
        std::cout << "\n\nConstraint Speeds" << std::endl;
        for (int j = 0; j < _nSecondaryCoord; j++) {
            double dq = parameters[_nActuators+j] - _init_secondary_values[j];
            double secondary_u = dq / _dt;
            std::cout << _parameter_names[_nActuators+j] << secondary_u << std::endl;
        }
        
        std::cout << "\n\nOptimization Constraints\n ";
        std::cout << "constraint = initial udot + udot - desired udot\n";
        std::cout << "name\t constraint \t initial udot \t udot \t desired udot \n";
        for (int i = 0; i < _nConstraints; i++)
        {
            std::cout << _constraint_names[i] << "\t " << constraints[i] << "\t " <<  _constraint_initial_udot[i] << "\t " << udots[i] << "\t " << _constraint_desired_udot[i] <<  std::endl;
        }

   
        std::cin.ignore();
    }
    /*SimTK::Vector actuator_params(_nActuators);
	SimTK::Vector secondary_coord_params(_nActuators);
	
	for (int i = 0; i < _nActuators; ++i) {
		actuator_params = parameters(i);
	}

	for (int j = 0; j < _nSecondaryCoord; ++j) {
		secondary_coord_params = parameters(_nActuators + j);
	}

	constraints = _actuator_unit_udot * actuator_params + _secondary_coord_unit_udot * secondary_coord_params + _damping_unit_udot * secondary_coord_params;*/
	
	//constraints = _constraint_matrix * parameters + _constraint_vector;

    /*
	//if (_verbose > 2) {
		if(false){
		std::cout << "Activation Values: ";
		i = 0;
		for (const auto& actuator : _model->getComponentList < ScalarActuator >()) {
			//actuator.getActuation(s)
			std::cout << actuator.getName() << " " << parameters[i] << std::endl;
			i++;
		}
		std::cout << "\n" << std::endl;
		
		std::cout << "\n\n\t\t Constraint Violations:\t\t Observed udot: \n";
		int i = 0;
		int j = 0;
		for (const Coordinate& coord : _model->getComponentList<Coordinate>()) {
			if (_primary_coords.findIndex(coord.getAbsolutePathString()) > -1 || 
				_secondary_coords.findIndex(coord.getAbsolutePathString()) > -1) {
				std::cout << i << " " << _constraint_names[i] << ":\t\t" << 
					constraints[i] << " \t\t" << _observed_udot[j] << std::endl;
				i++;
			}
			j++;
		}		
	}*/
	
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

        //Reserve Actuators
        for (int j = 0; j < _nReserveActuators; ++j) {
            jac(i,p) = _optimalForce[p] * _reserve_unit_udot(i, j);
            p++;
        }

        //Secondary Coordinates

        for (int j = 0; j < _nSecondaryCoord; ++j) {
            jac(i,p) = _secondary_coord_unit_udot(i,j);
            jac(i,p) += _secondary_damping_unit_udot(i,j)/_dt;
            p++;
        }
    }
    
    if (false) {
        std::cout << "\n\nConstraint Gradients:\n ";
        for (int i = 0; i < _nConstraints; ++i) {
            std::cout << _constraint_names[i] << std::endl;
            for (int j = 0; j < _nParameters; ++j) {
                std::cout << jac(i, j) << "   ";
            }
            std::cout << std::endl << std::endl;
        }
        std::cin.ignore();        
    }
    
    //jac = _constraint_matrix;

/*
	if (_verbose > 1) {

		std::cout << "Constraint Gradients:\n ";
		i = 0;
		for (const Coordinate& coord : _model->getComponentList<Coordinate>()) {

			if (!coord.isConstrained(_state)) {
				std::cout << coord.getName() << ":";
				for (int j = 0; j < _nActuators; ++j) {
					std::cout << " " << jac(i, j);
				}
				std::cout << "\n ";
				i++;
			}
		}

	}
	*/

    return 0;
}
//=============================================================================
// ACCELERATION
//=============================================================================
//
void ComakTarget::computeSimulatedAcceleration(SimTK::State s, const SimTK::Vector &parameters, SimTK::Vector &sim_udot) const
{
    if (false) {
        std::cout << "Initial Parameters" << std::endl;
        for (int i = 0; i < _nParameters; i++) {
            std::cout << _parameter_names[i] << ": " << parameters[i] << std::endl;
        }
    }

    //Apply Muscle Forces
	int j = 0;
	for (Muscle &msl : _model->updComponentList<Muscle>()) {
		msl.overrideActuation(s, true);
		double force = _optimalForce[j] * parameters[j];
		msl.setOverrideActuation(s,force);		
        j++;
    }
    //Apply Reserve Forces
    for (CoordinateActuator &coord_act : _model->updComponentList<CoordinateActuator>()) {
		coord_act.overrideActuation(s, true);
		double force = _optimalForce[j] * parameters[j];
		coord_act.setOverrideActuation(s,force);		
        j++;
    }

    //_model->realizeAcceleration(s);

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
    _reserve_unit_udot.resize(_nConstraints, _nReserveActuators);
	_secondary_coord_unit_udot.resize(_nConstraints, _nSecondaryCoord);
	_secondary_damping_unit_udot.resize(_nConstraints, _nSecondaryCoord);
	_secondary_coord_unit_energy.resize(_nSecondaryCoord);
       
	_msl_unit_udot = -1;
    _reserve_unit_udot = -1;
	_secondary_coord_unit_udot = -1;
	_secondary_damping_unit_udot = -1;
    _secondary_coord_unit_energy = -1;

   //Apply Muscle Forces
	int j = 0;
	for (Muscle &msl : _model->updComponentList<Muscle>()) {
		msl.overrideActuation(s, true);
		double force = _optimalForce[j] * parameters[j];
		msl.setOverrideActuation(s,force);		
        j++;
    }
    //Apply Reserve Forces
    for (CoordinateActuator &coord_act : _model->updComponentList<CoordinateActuator>()) {
		coord_act.overrideActuation(s, true);
		double force = _optimalForce[j] * parameters[j];
		coord_act.setOverrideActuation(s,force);		
        j++;
    }

	//_model->realizeAcceleration(s);

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
    
    //Current Ligament Forces
    int nLig = 0;
    for (Blankevoort1991Ligament& lig : _model->updComponentList<Blankevoort1991Ligament>()) {
        nLig++;
    }

	//Compute Muscle Unit Udot
	j = 0;
	for (Muscle &msl : _model->updComponentList<Muscle>()) {
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

    //Compute Reserve Unit Udot
    j = 0;
    for (CoordinateActuator &reserve : _model->updComponentList<CoordinateActuator>()) {
		double force = parameters[_nMuscles + j] * _optimalForce[_nMuscles + j];		
		reserve.setOverrideActuation(s, force + 1.0);

		_model->realizeAcceleration(s);
		
		int k = 0;
		for (const Coordinate& coord : _model->updComponentList<Coordinate>()) {
			if (_primary_coords.findIndex(coord.getAbsolutePathString()) > -1 ||
				_secondary_coords.findIndex(coord.getAbsolutePathString()) > -1) {

				_reserve_unit_udot(k, j) = (coord.getAccelerationValue(s) - _constraint_initial_udot(k));
				k++;
			}
		}		
		reserve.setOverrideActuation(s, force);

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

        //Ligament Force Change
        /*i = 0;
        //std::cout << _secondary_coords[j] << std::endl;
        for (Blankevoort1991Ligament& lig : _model->updComponentList<Blankevoort1991Ligament>()) {
            //std::cout << lig.getName() << lig.getDynamicQuantities(s, "force_total") - lig_tot_frc(i) << std::endl;
            i++;
        }*/

		value = parameters(_nActuators + j);
		_model->updComponent<Coordinate>(_secondary_coords[j]).setValue(s, value, true);
	}
    _model->realizeAcceleration(s);
	//Compute COMAK damping unit udot
    SimTK::Vector org_reserve_frc(_nSecondaryCoord,0.0);

    for (int j = 0; j < _nSecondaryCoord; ++j) {
        std::string coord_name = _model->updComponent<Coordinate>(_secondary_coords[j]).getName();
        org_reserve_frc(j) = _model->updComponent<CoordinateActuator>(coord_name + "_reserve").getActuation(s);
    }

	for (int j = 0; j < _nSecondaryCoord; ++j) {
		std::string coord_name = _model->updComponent<Coordinate>(_secondary_coords[j]).getName();
			
		CoordinateActuator& reserve = _model->updComponent<CoordinateActuator>(coord_name + "_reserve");

		//_model->realizeAcceleration(s);
		//double org_frc = reserve.getActuation(s);

		reserve.setOverrideActuation(s, org_reserve_frc(j) + 1.0);

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

		reserve.setOverrideActuation(s, org_reserve_frc(j));		
	}

	_model->realizeAcceleration(s);

    if (false) {        
        
        std::cout << "\n\nMuscle Udots\n" << std::endl;
        std::cout << std::setprecision(5) << std::setw(20) << " ";
		for (const Coordinate& coord : _model->getComponentList<Coordinate>()) {
			if (_primary_coords.findIndex(coord.getAbsolutePathString()) > -1 ||
				_secondary_coords.findIndex(coord.getAbsolutePathString()) > -1) {
				std::cout << std::setw(13) << coord.getName() << "  ";
			}
		}
		std::cout << std::endl;
        for (int j = 0; j < _nMuscles; ++j) {
            std::cout << std::setw(20) << _parameter_names[j];
            for (int i = 0; i < _nConstraints; ++i) {
                std::cout << std::setw(13) << _msl_unit_udot(i, j) << "  ";
            }
            std::cout << std::endl;
        }

        std::cout << "\n\nReserve Udots\n" << std::endl;
        std::cout << std::setprecision(5) << std::setw(20) << " ";
		for (const Coordinate& coord : _model->getComponentList<Coordinate>()) {
			if (_primary_coords.findIndex(coord.getAbsolutePathString()) > -1 ||
				_secondary_coords.findIndex(coord.getAbsolutePathString()) > -1) {
				std::cout << std::setw(13) << coord.getName() << "  ";
			}
		}
		std::cout << std::endl;
        for (int j = 0; j < _nReserveActuators; ++j) {
            std::cout << std::setw(20) << _parameter_names[_nMuscles + j];
            for (int i = 0; i < _nConstraints; ++i) {
                std::cout << std::setw(13) << _reserve_unit_udot(i, j) << "  ";
            }
            std::cout << std::endl;
        }

        std::cout << "\n\nSecondary Coordinate Udots\n" << std::endl;
        std::cout << std::setprecision(5) << std::setw(20) << " ";
		for (const Coordinate& coord : _model->getComponentList<Coordinate>()) {
			if (_primary_coords.findIndex(coord.getAbsolutePathString()) > -1 ||
				_secondary_coords.findIndex(coord.getAbsolutePathString()) > -1) {
				std::cout << std::setw(13) << coord.getName() << "  ";
			}
		}
		std::cout << std::endl;
        for (int j = 0; j < _nSecondaryCoord; ++j) {
            std::cout << std::setw(20) << _parameter_names[_nActuators + j];
            for (int i = 0; i < _nConstraints; ++i) {
                std::cout << std::setw(13) << _secondary_coord_unit_udot(i, j) << "  ";
            }
            std::cout << std::endl;
        }

        std::cout << "\n\nSecondary damping Udots\n" << std::endl;
        std::cout << std::setprecision(5) << std::setw(20) << " ";
		for (const Coordinate& coord : _model->getComponentList<Coordinate>()) {
			if (_primary_coords.findIndex(coord.getAbsolutePathString()) > -1 ||
				_secondary_coords.findIndex(coord.getAbsolutePathString()) > -1) {
				std::cout << std::setw(13) << coord.getName() << "  ";
			}
		}
		std::cout << std::endl;
        for (int j = 0; j < _nSecondaryCoord; ++j) {
            std::cout << std::setw(20) << _parameter_names[_nActuators + j];
            for (int i = 0; i < _nConstraints; ++i) {
                std::cout << std::setw(13) << _secondary_damping_unit_udot(i, j) << "  ";
            }
            std::cout << std::endl;
        }
        
        std::cout << "\n\nSecondary Coordinate Unit Energy\n" << std::endl;
        std::cout << std::setprecision(5) << std::setw(20) << " ";
		std::cout << std::endl;
        for (int j = 0; j < _nSecondaryCoord; ++j) {
            std::cout << std::setw(20) << _parameter_names[_nActuators + j] << std::setw(13) << _secondary_coord_unit_energy(j) << std::endl;
        }

        
    }
    
    /*
        std::cout << std::setprecision(5) << std::setw(20) << " ";
		for (const Coordinate& coord : _model->getComponentList<Coordinate>()) {
			if (_primary_coords.findIndex(coord.getAbsolutePathString()) > -1 ||
				_secondary_coords.findIndex(coord.getAbsolutePathString()) > -1) {
				std::cout << std::setw(13) << coord.getName() << "  ";
			}
		}
		std::cout << std::endl;
		for (int i = 0; i < _nParameters; ++i) {
			std::cout << std::setw(20) << _parameter_names[i];
			for (int j = 0; j < _nConstraints; ++j) {
				std::cout << std::setw(13) << _constraint_matrix(j, i) << "  ";
			}
			std::cout << std::endl;
		}*/
}

void ComakTarget::setParameterBounds(double scale) {

	SimTK::Vector lower_bounds(_nParameters), upper_bounds(_nParameters);
	int p = 0;
	for (Muscle &msl : _model->updComponentList<Muscle>()) {
		double min_value = msl.getMinControl();
		double max_value = msl.getMaxControl();

		lower_bounds(p) = min_value;
		upper_bounds(p) = max_value;
		p++;
	}
    for (CoordinateActuator &reserve : _model->updComponentList<CoordinateActuator>()) {
		double min_value = reserve.getMinControl()*scale;
		double max_value = reserve.getMaxControl()*scale;

		lower_bounds(p) = min_value;
		upper_bounds(p) = max_value;
		p++;
	}

	for (int j = 0; j < _nSecondaryCoord; j++) {
		Coordinate& coord = _model->updComponent<Coordinate>(_secondary_coords[j]);
		if (coord.getMotionType() == Coordinate::MotionType::Rotational) {
			lower_bounds(p) = _init_parameters[p] -_max_change_rotation*scale;
			upper_bounds(p) = _init_parameters[p] + _max_change_rotation*scale;
		}
		else if (coord.getMotionType() == Coordinate::MotionType::Translational) {
			lower_bounds(p) = _init_parameters[p] -_max_change_translation*scale;
			upper_bounds(p) = _init_parameters[p] + _max_change_translation*scale;
		}
		else {
			lower_bounds(p) = _init_parameters[p] -_max_change_translation*scale;
			upper_bounds(p) = _init_parameters[p] + _max_change_translation*scale;
		}
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

