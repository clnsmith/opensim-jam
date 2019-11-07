#ifndef OPENSIM_COMAK_TARGET_H_
#define OPENSIM_COMAK_TARGET_H_
/* -------------------------------------------------------------------------- *
 *                              COMAKTarget.h                                 *
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
//#include "osimAnalysesDLL.h"
#include "OpenSim/Common/Array.h"
#include "COMAKTool.h"
#include <simmath/Optimizer.h>

//=============================================================================
//=============================================================================
namespace OpenSim { 

/**

 */
class ComakTarget : public SimTK::OptimizerSystem
{


//=============================================================================
// METHODS
//=============================================================================
public:
	ComakTarget(SimTK::State s, Model* aModel, const SimTK::Vector& observed_udot, 
		const SimTK::Vector& init_parameters,
		Array<std::string> primary_coords, Array<std::string> secondary_coords,
		bool useMusclePhysiology = false);


	void initialize();

    //--------------------------------------------------------------------------
    // REQUIRED OPTIMIZATION TARGET METHODS
    //--------------------------------------------------------------------------
    int objectiveFunc(const SimTK::Vector &x, bool new_coefficients, SimTK::Real& rP) const override;
    int gradientFunc(const SimTK::Vector &x, bool new_coefficients, SimTK::Vector &gradient) const override;
    int constraintFunc(const SimTK::Vector &x, bool new_coefficients, SimTK::Vector &constraints) const override;
    int constraintJacobian(const SimTK::Vector &x, bool new_coefficients, SimTK::Matrix &jac) const override;

	//Set
	void setModel(Model& aModel){
		_model = &aModel;
	}

	void setCostFunctionWeight(SimTK::Vector weight) {
		_weight = weight;
	}

	void setDesiredActivation(SimTK::Vector desired_act) {
		_desired_act = desired_act;

	}

	void  setMaxChangeRotation(double max_change_rotation) {
		_max_change_rotation = max_change_rotation;
	}
		
	void setMaxChangeTranslation(double max_change_translation) {
		_max_change_translation = max_change_translation;
	}
		
	void setUdotTolerance(double udot_tolerance) {
		_udot_tolerance = udot_tolerance;
	}

	void setUnitUdotEpsilon(double unit_udot_epsilon) {
		_unit_udot_epsilon = unit_udot_epsilon;
	};

	void setDT(double dt) {
		_dt = dt;
	};

	void setOptimalForces(SimTK::Vector optimal_force) {
		 _optimalForce = optimal_force;
	}

	void setParameterScale(SimTK::Vector parameter_scale) {
		_parameter_scale = parameter_scale;
	}

	void setObservedSpeeds(SimTK::Vector speeds) {
		_observed_u = speeds;
	}

	void setSecondaryIndex(Array<int> index) {
		_secondary_index = index;
	}

    void setMuscleVolumes(SimTK::Vector msl_volumes) {
        _msl_volumes = msl_volumes;
    }

	void setComakDampingMultiplier(double damping_multiplier) {
		_damping_multiplier = damping_multiplier;
	}

    void setSecondaryCoordinateDamping(SimTK::Vector& secondary_coord_damping) {
        _secondary_coord_damping = secondary_coord_damping;
    }

    void setContactEnergyWeight(double contact_energy_weight) {
        _contact_energy_weight = contact_energy_weight;
    }

    void setPrevSecondaryValues(SimTK::Vector prev_secondary_values) {
        _prev_secondary_values = prev_secondary_values;
    }

	//Helper
	void computeSimulatedAcceleration(SimTK::State s, const SimTK::Vector &parameters, SimTK::Vector& sim_udot) const;
	void computeUnitUdot(SimTK::State s, const SimTK::Vector& parameters);
	void precomputeConstraintMatrix();
	void setParameterBounds(double scale);
    void printPerformance(SimTK::Vector parameters);
private:

	//=============================================================================
	// DATA
	//=============================================================================
public:

private:
	Model * _model;
	SimTK::State _state;
	SimTK::Vector _optimalForce;
	SimTK::Vector _init_parameters;
	bool _useMusclePhysiology;
	double _activationExponent;
	SimTK::Vector _observed_udot;
	SimTK::Vector _observed_u;
	int _nSecondaryCoord;
	int _nPrimaryCoord;
	int _nActuators;
	int _nConstraints;
	int _nParameters;
	int _nCoordinates;
    int _nMuscles;
    int _nReserveActuators;

    SimTK::Matrix _msl_unit_udot;
    SimTK::Matrix _reserve_unit_udot;
    SimTK::Matrix _secondary_coord_unit_udot;
    SimTK::Matrix _secondary_damping_unit_udot;
    SimTK::Vector _secondary_coord_unit_energy;

	SimTK::Vector _init_secondary_values;
    SimTK::Vector _prev_secondary_values;
    SimTK::Vector _secondary_coord_damping;

	Array<std::string> _primary_coords;
	Array<std::string> _secondary_coords;
	Array<std::string> _parameter_names;
	Array<std::string> _constraint_names;

	int _verbose;

    SimTK::Vector _msl_volumes;
    SimTK::Vector _weight;
	SimTK::Vector _desired_act;

	double _max_change_rotation;
	double _max_change_translation;
	double _udot_tolerance;
	double _unit_udot_epsilon;
	double _dt;
	double _damping_multiplier;
    double _contact_energy_weight;

	SimTK::Vector _gradient_scale;
	SimTK::Vector _parameter_scale;
	Array<int> _secondary_index;

    SimTK::Vector _initial_udot;
    SimTK::Vector _constraint_initial_udot;
    SimTK::Vector _constraint_desired_udot;
protected:
};

}; //namespace

#endif // OPENSIM_COMAK_TARGET_H_
