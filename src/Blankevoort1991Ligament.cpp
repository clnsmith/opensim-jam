/* -------------------------------------------------------------------------- *
 *                         Blankevoort1991Ligament.cpp                        *
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

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/PointForceDirection.h>
#include "Blankevoort1991Ligament.h"
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Common/PolynomialFunction.h>

using namespace SimTK;
using namespace OpenSim;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor
 */
Blankevoort1991Ligament::Blankevoort1991Ligament() : Force()
{
    constructProperties();
	setNull();
}
Blankevoort1991Ligament::Blankevoort1991Ligament(std::string name, const PhysicalFrame& frame1, Vec3 point1,
	const PhysicalFrame& frame2, Vec3 point2) : Blankevoort1991Ligament()
{
	setName(name);
	upd_GeometryPath().appendNewPathPoint("p1", frame1, point1);
	upd_GeometryPath().appendNewPathPoint("p2", frame2, point2);
}

Blankevoort1991Ligament::Blankevoort1991Ligament(std::string name, const PhysicalFrame& frame1, Vec3 point1,
	const PhysicalFrame& frame2, Vec3 point2,
	double linear_stiffness, double slack_length) :
	Blankevoort1991Ligament(name,frame1, point1, frame2, point2)
{
	set_linear_stiffness(linear_stiffness);
	set_slack_length(slack_length);	
}
//_____________________________________________________________________________




void Blankevoort1991Ligament::setNull()
{
	setAuthors("Colin Smith");
	setReferences(
	"Smith, C.R., Lenhart, R.L., Kaiser, J., Vignos, M.F. and Thelen, D.G.,"
	"(2016). Influence of ligament properties on tibiofemoral mechanics"
	" in walking. J Knee Surg, 29(02), 99-106.\n\n"
	
	"Wismans, J.A.C., Veldpaus, F., Janssen, J., Huson, A. and Struben, P.," 
	"(1980). A three-dimensional mathematical model of the knee-joint. "
	"J Biomech, 13(8), 677-685.\n\n"

	"Blankevoort, L. and Huiskes, R., (1991)."
	"Ligament-bone interaction in a three-dimensional model of the knee."
	"J Biomech Eng, 113(3), 263-269"
	);
}

void Blankevoort1991Ligament::constructProperties()
{
	constructProperty_GeometryPath(GeometryPath());
	constructProperty_linear_stiffness(0.0);
	constructProperty_transition_strain(0.06);
	constructProperty_normalized_damping_coefficient(0.003);
	constructProperty_slack_length(0.0);
	
}

void Blankevoort1991Ligament::extendFinalizeFromProperties()
{
	Super::extendFinalizeFromProperties();

	//Check that properties are valid
	OPENSIM_THROW_IF_FRMOBJ(get_slack_length() < 0.,
        InvalidPropertyValue, getProperty_slack_length().getName(),
        "Slack Length cannot be less than 0");

	OPENSIM_THROW_IF_FRMOBJ(get_linear_stiffness() < 0.,
        InvalidPropertyValue, getProperty_linear_stiffness().getName(),
        "Linear Stiffness cannot be less than 0");

	OPENSIM_THROW_IF_FRMOBJ(get_normalized_damping_coefficient() < 0.,
        InvalidPropertyValue, getProperty_normalized_damping_coefficient().getName(),
        "Normalized Damping Coefficient cannot be less than 0");

	OPENSIM_THROW_IF_FRMOBJ(get_transition_strain() < 0.,
        InvalidPropertyValue, getProperty_transition_strain().getName(),
        "Transistion Strain cannot be less than 0");

	//Set Default Ligament Color
	GeometryPath& path = upd_GeometryPath();
    path.setDefaultColor(SimTK::Vec3(0.99, 0.42, 0.01));

    //Set Force Functions
    //toe region F=
    SimTK::Vector coefficients(3, 0.0);
    coefficients.set(2, 0.5*get_linear_stiffness() / get_transition_strain());
    _springToeForceFunction = PolynomialFunction(coefficients);
	
    //linear region F=k*(e-e_t/2)
    double slope = get_linear_stiffness();
    double intercept = -get_transition_strain() / 2;
    _springLinearForceFunction = LinearFunction(slope, intercept);
}

void Blankevoort1991Ligament::extendAddToSystem(SimTK::MultibodySystem& system) const
{
	Super::extendAddToSystem(system);

    addCacheVariable<double>("strain", 0.0, SimTK::Stage::Position);
	addCacheVariable<double>("strain_rate", 0.0, SimTK::Stage::Velocity);
	addCacheVariable<double>("force_spring",0.0, SimTK::Stage::Position);
	addCacheVariable<double>("force_damping", 0.0, SimTK::Stage::Velocity);
	addCacheVariable<double>("force_total", 0.0, SimTK::Stage::Velocity);
}


void Blankevoort1991Ligament::setSlackLengthFromReferenceStrain(double reference_strain, const SimTK::State state) {
	double reference_length = computeReferenceLength(state);
	double slack_length = reference_length / (1.0 + reference_strain);
    set_slack_length(slack_length);
};

void Blankevoort1991Ligament::setSlackLengthFromReferenceForce(double reference_force, const SimTK::State state) {
	double reference_strain = computeSpringStrain(reference_force);
	setSlackLengthFromReferenceStrain(reference_strain, state);
};

double Blankevoort1991Ligament::computeReferenceLength(SimTK::State state) const {
	getModel().realizePosition(state);

	return get_GeometryPath().getLength(state);
}


double Blankevoort1991Ligament::computeReferenceStrain(const SimTK::State& state) const {
	double ref_length = computeReferenceLength(state);
	double ref_strain = ref_length / get_slack_length() - 1;

	return ref_strain;
}

double Blankevoort1991Ligament::computeReferenceForce(const SimTK::State& state) const {
	double ref_strain = computeReferenceStrain(state);
	double ref_force = computeSpringForce(ref_strain);

	return ref_force;
}

double Blankevoort1991Ligament::computeSpringForce(double strain) const 
{
	double force_spring;
    //slack region
	if (strain <= 0) {
		force_spring = 0;
	}
    //toe region
	else if ((strain > 0) && (strain < (get_transition_strain()))) {
		//force_spring = 0.5 * get_linear_stiffness() * strain * strain / get_transition_strain();
        _springToeForceFunction.calcValue(SimTK::Vector(1,strain));
	}
    //linear region F=k*(e-e_t/2)
	else if (strain >= (get_transition_strain())) {
		//force_spring = get_linear_stiffness() * (strain - (get_transition_strain() / 2));
        _springLinearForceFunction.calcValue(SimTK::Vector(1,strain));
	}
	return force_spring;
}

double Blankevoort1991Ligament::computeSpringStrain(double force) const {
	double transistion_force = get_transition_strain() * get_linear_stiffness();

	double strain;
	if (force < 0.0) {
		strain = 0.0;
	}
	else if (force > transistion_force) {
		strain = force / get_linear_stiffness() + get_transition_strain() / 2;
	}
	else {
		strain = sqrt(2 * get_transition_strain() * force / get_linear_stiffness());
	}
	
	return strain;
}

//=============================================================================
// SCALING
//=============================================================================

void Blankevoort1991Ligament::extendPostScale(const SimTK::State& s, const ScaleSet& scaleSet)
{
    Super::extendPostScale(s, scaleSet);

	GeometryPath& path = upd_GeometryPath();
	double slack_length = get_slack_length();
    if (path.getPreScaleLength(s) > 0.0)
    {
        double scaleFactor = path.getLength(s) / path.getPreScaleLength(s);
		set_slack_length(scaleFactor * slack_length);

        // Clear the pre-scale length that was stored in the GeometryPath.
        path.setPreScaleLength(s, 0.0);
    }
}

//=============================================================================
// GET
//=============================================================================

double Blankevoort1991Ligament::getStrain(const SimTK::State& s) const{
    if(!isCacheVariableValid(s,"strain")){
        double length = getLength(s);
        double strain = (length - get_slack_length())/get_slack_length();
        
        setCacheVariableValue<double>(s, "strain", strain);
        return strain;
    }
    return getCacheVariableValue<double>(s, "strain");
}

double Blankevoort1991Ligament::getStrainRate(const SimTK::State& s) const {        
   if(!isCacheVariableValid(s,"strain_rate")){
        double lengthening_speed = getLengtheningSpeed(s);
        double strain_rate = lengthening_speed / get_slack_length();

        setCacheVariableValue<double>(s, "strain", strain_rate);
        return strain_rate;
    }
    return getCacheVariableValue<double>(s, "strain");
}

double Blankevoort1991Ligament::getLength(const SimTK::State& s) const {
    return get_GeometryPath().getLength(s);
}

double Blankevoort1991Ligament::getLengtheningSpeed(const SimTK::State& s) const {
    return get_GeometryPath().getLengtheningSpeed(s);
}

double Blankevoort1991Ligament::getSpringForce(const SimTK::State& s) const {
    if(!isCacheVariableValid(s,"force_spring")){
        double strain = getStrain(s);
        double force_spring = computeSpringForce(strain);

        setCacheVariableValue<double>(s, "force_spring", force_spring);
        return force_spring;
    }
    return getCacheVariableValue<double>(s, "force_spring");
}

double Blankevoort1991Ligament::getDampingForce(const SimTK::State& s) const {
    if(!isCacheVariableValid(s,"force_damping")){
        double strain = getStrain(s);
        double strain_rate = getStrainRate(s);
        double force_damping = 0.0;

        if (strain > 0) {
	        force_damping = get_linear_stiffness()*get_normalized_damping_coefficient()*strain_rate;
        }
        else {
	        force_damping = 0.0;
        }

        //Phase-out damping as strain goes to zero with smooth-step function
        SimTK::Function::Step step(0, 1, 0, 0.01);
        SimTK::Vector in_vec(1,strain);
        force_damping = force_damping*step.calcValue(in_vec);


        setCacheVariableValue<double>(s, "force_damping", force_damping);
        return force_damping;
    }
    return getCacheVariableValue<double>(s, "force_damping");
}

double Blankevoort1991Ligament::getTension(const SimTK::State& s) const
{
	if (get_appliesForce()) {
		if(!isCacheVariableValid(s,"force_spring")){
            double force_total = getDampingForce(s) + getSpringForce(s);

            // make sure the ligament is only acting in tension
	        if (force_total < 0.0) {
		        force_total = 0.0;
	        }

            setCacheVariableValue<double>(s, "force_total", force_total);
            return force_total;
        }
        return getCacheVariableValue<double>(s, "force_total");
	}
	else {
		return 0.0;
	}
}
//=============================================================================
// COMPUTATION
//=============================================================================

double Blankevoort1991Ligament::computeMomentArm(const SimTK::State& s, Coordinate& aCoord) const
{
	return get_GeometryPath().computeMomentArm(s, aCoord);
}

void Blankevoort1991Ligament::computeForce(const SimTK::State& s,
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
							  SimTK::Vector& generalizedForces) const
{
	// total force
	double force_total = getTension(s);

	OpenSim::Array<PointForceDirection*> PFDs;
	get_GeometryPath().getPointForceDirections(s, &PFDs);

	for (int i=0; i < PFDs.getSize(); i++) {
		applyForceToPoint(s, PFDs[i]->frame(), PFDs[i]->point(),
                          force_total*PFDs[i]->direction(), bodyForces);
	}
	
	for(int i=0; i < PFDs.getSize(); i++)
		delete PFDs[i];
}

double Blankevoort1991Ligament::computePotentialEnergy(const SimTK::State& state) const {
	double strain = getCacheVariableValue<double>(state, "strain");
	double lin_stiff = get_linear_stiffness();
	double trans_strain = get_transition_strain();
	double slack_len = get_slack_length();

	if (strain < trans_strain) {
		return 1 / 6 * lin_stiff / trans_strain*pow(strain, 3);
	}
	else {
		return 1 / 6 * lin_stiff / trans_strain*pow(trans_strain, 3)+1/2*lin_stiff*strain*(strain-trans_strain);
	}

}


//=============================================================================
// Reporting
//=============================================================================
	OpenSim::Array<std::string> Blankevoort1991Ligament::getRecordLabels() const {
		OpenSim::Array<std::string> labels("");
		labels.append(getName()+".force_spring");
		labels.append(getName()+".force_damping");
		labels.append(getName()+".force_total");
		labels.append(getName()+".length");
		labels.append(getName()+".lengthening_speed");
		labels.append(getName() + ".strain");
		labels.append(getName() + ".strain_rate");
		return labels;
	}

	OpenSim::Array<double> Blankevoort1991Ligament::getRecordValues(const SimTK::State& s) const {
		OpenSim::Array<double> values(1);

		// Report values
		values.append(getCacheVariableValue<double>(s, "force_spring"));
		values.append(getCacheVariableValue<double>(s, "force_damping"));
		values.append(getCacheVariableValue<double>(s, "force_total"));
		values.append(getLength(s));
		values.append(getLengtheningSpeed(s));
		values.append(getCacheVariableValue<double>(s, "strain"));
		values.append(getCacheVariableValue<double>(s, "strain_rate"));
		return values;
	}

	/*void Blankevoort1991Ligament::printPropertiesToConsole() {
		std::string def_prop = get_defining_slack_length_property();
		double lin_stiff = get_linear_stiffness();
		double ref_strain = get_reference_strain();
		double ref_force = get_reference_force();
		double slack_len = get_slack_length();
		double damp_c = get_normalized_damping_coefficient();
		double trans_strain = get_transition_strain();

		std::cout << "Blankevoort1991Ligament: " << getName() << std::endl;
		std::cout << "==============================" << std::endl;
		std::cout << "Linear Stiffness: " << lin_stiff << std::endl;
		std::cout << "Defining Slack Length Property:" << def_prop << std::endl;
		std::cout << "Reference Strain: " << ref_strain << std::endl;
		std::cout << "Reference Force: " << ref_force << std::endl;
		std::cout << "Slack Length: " << slack_len << std::endl;
		std::cout << "Transition Strain: " << trans_strain << std::endl;
		std::cout << "Normalized Damping Coeff: " << damp_c<< std::endl;
		std::cout << std::endl;


	}*/
