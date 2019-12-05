#ifndef OPENSIM_JOINT_MECHANICS_TOOL_H_
#define OPENSIM_JOINT_MECHANICS_TOOL_H_
/* -------------------------------------------------------------------------- *
 *                            JointMechanicsTool.h                            *
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
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim\Simulation\Model\Model.h>
#include "Smith2018ArticularContactForce.h"
#include "H5FileAdapter.h"
#include "osimPluginDLL.h"
#include "H5Cpp.h"
#include "hdf5_hl.h"
#include "OpenSim/Simulation/StatesTrajectory.h"

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
class OSIMPLUGIN_API JointMechanicsTool : public Object {

    OpenSim_DECLARE_CONCRETE_OBJECT(JointMechanicsTool, Object);

//=============================================================================
// PROPERTIES
//=============================================================================
public:
	OpenSim_DECLARE_PROPERTY(model_file, std::string,
		"Path to .osim file for analysis.")
	OpenSim_DECLARE_PROPERTY(coordinates_file, std::string,
		"Motion file (.mot) or storage file (.sto) containing the time history"
        " of the generalized coordinates for the model.")
	OpenSim_DECLARE_PROPERTY(results_directory, std::string,
		"Folder where the results files are output.")
	OpenSim_DECLARE_PROPERTY(results_file_basename, std::string,
		"Prefix to each results file name.")
	OpenSim_DECLARE_PROPERTY(start_time, double,
		"Time to start analysis. Set to -1 to use initial frame in "
        "coordinates_file")
	OpenSim_DECLARE_PROPERTY(stop_time, double,
		"Time to stop analysis. Set to -1 to use last frame in coordinates_file")
	OpenSim_DECLARE_PROPERTY(resample_step_size, double,
		"Time step size to report results, set to -1 to use steps directly from coordinates_file")
	OpenSim_DECLARE_PROPERTY(normalize_to_cycle, bool,
		"Resample to 101 equally spaced time steps. Note: If true, this overrides resample_step_size.")
	OpenSim_DECLARE_PROPERTY(smoothing_spline_frequency, double,
		"Apply a 5 order smoothing spline to input coordinates, set to -1 to use raw data."
		"NOTE: lowpass filter is NOT applied if smoothing_spline_frequency > -1")
	OpenSim_DECLARE_PROPERTY(smoothing_spline_order, int,
        "Order of the smoothing spline.")
	OpenSim_DECLARE_PROPERTY(lowpass_filter_frequency, double,
		"Apply IIR lowpass butterworth filter the input coordinates,  set to -1 to use raw data")
	OpenSim_DECLARE_PROPERTY(print_processed_kinematcs, bool,
		"Print a .sto file with the processed kinematics used for posing the analysis.")
    OpenSim_DECLARE_LIST_PROPERTY(contact_names, std::string, 
        "Names of Smith2018ArticularContactForce components to be recorded."
        "Options: 'none','all', or a list of Smith2018ArticularContactForce "
        "component names.")
    OpenSim_DECLARE_LIST_PROPERTY(tri_output_data_names, std::string, 
        "Names of Smith2018ArticularContactForce outputs for every triangle " 
        "in contact meshes."
        "Options: 'pressure','proximity','potential_energy','all'")
	OpenSim_DECLARE_PROPERTY(write_vtp_files,bool,
		"Write .vtp files visualization.")
	OpenSim_DECLARE_PROPERTY(vtp_origin,std::string,
		"Origin for mesh motion in .vtp files."
        "Options: 'ground' or 'body_name' for dynamic files, 'static' for the" 
        " contact meshes in local reference frame.")
    OpenSim_DECLARE_PROPERTY(vtp_frame,std::string,
		"Reference frame for mesh motion in .vtp files."
        "Options: 'ground' or 'body_name' for dynamic files, 'static' for the"
        " contact meshes in local reference frame.")
	OpenSim_DECLARE_PROPERTY(vtp_contact, bool,
		"Generate .vtp files for Smith2018ArticularContactForce " 
        "of contact meshes with output data for visualization.")
    OpenSim_DECLARE_LIST_PROPERTY(vtp_attached_geometries, std::string,
		"Generate .vtp files for attached_geometry meshes for visualization." 
		"Options: 'none','all', or a list of frame names.")
	OpenSim_DECLARE_PROPERTY(vtp_ligaments, bool,
		"Write ligament lines of action and output data to .vtp files")
    OpenSim_DECLARE_PROPERTY(vtp_muscles, bool,
		"Write muscle lines of action and output data to .vtp files")
	OpenSim_DECLARE_LIST_PROPERTY(vtp_contact_properties, std::string,
		"Add triangle contact property value maps to vtp files."
		"Options: 'none','thickness','elastic modulus','poisson ratio'"
        "'area' or 'all'")
	OpenSim_DECLARE_PROPERTY(write_h5_file, bool,
		"Write binary .h5 file")
	OpenSim_DECLARE_PROPERTY(h5_raw_contact_data, bool,
		"Write detailed contact data to .h5 file")
	OpenSim_DECLARE_PROPERTY(h5_summary_contact_data, bool,
		"Write summary statistics of contact to .h5 file.")
	OpenSim_DECLARE_PROPERTY(h5_regional_summary_contact, bool,
		"Write medial-lateral regional summary statistics to .h5 file")
	OpenSim_DECLARE_PROPERTY(h5_states_data, bool,
		"Write states data to .h5 file")
	OpenSim_DECLARE_PROPERTY(h5_kinematics_data, bool,
			"Write kinematics data to .h5 file")
	OpenSim_DECLARE_PROPERTY(h5_muscle_data,bool,
		"Write muscle analysis data to .h5 file")
	OpenSim_DECLARE_PROPERTY(h5_ligament_data, bool,
		"Write Blankevoort1991Ligament data to h5 file")


//=============================================================================
// METHODS
//=============================================================================
public:
	JointMechanicsTool();
	JointMechanicsTool(std::string settings_file);
	JointMechanicsTool(Model *aModel,
		std::string coordinates_file, std::string results_dir);

	void setModel(Model& aModel);
	void loadModel(const std::string &aToolSetupFileName);
	void run();

	int printResults(const std::string &aBaseName, const std::string &aDir);

private:
    void setNull();
    void constructProperties();
    
	void initializeSettings();
	void formQandUMatrixFromFile();
    
	void setModelingOptions(SimTK::State& s);
	int record(const SimTK::State& s, const int frame_num);
	
	
	void writeVTPFile(const std::string& mesh_name,
		const std::vector<std::string>& contact_names, bool isDynamic);

	void writeAttachedGeometryVTPFiles(bool isDynamic);

    void writeLineVTPFiles(std::string line_name,
        const SimTK::Vector& nPoints, const SimTK::Matrix_<SimTK::Vec3>& path_points,
        const std::vector<std::string>& output_double_names, const SimTK::Matrix& output_double_values);
	void writeH5File(const std::string &aBaseName, const std::string &aDir);
    void writeBlenderCSVFile();

	void setupDynamicVertexLocationStorage();
	void setupLigamentStorage();
    void setupMuscleStorage();
    void setupCoordinateStorage();
    void setupContactStorage(const SimTK::State& state);
    void getGeometryPathPoints(const SimTK::State& s, const GeometryPath& geoPath, SimTK::Vector_<SimTK::Vec3>& path_points, int& nPoints);
    void JointMechanicsTool::collectMeshContactOutputData(const std::string& mesh_name,
        std::vector<SimTK::Matrix>& faceData, std::vector<std::string>& faceDataNames,
        std::vector<SimTK::Matrix>& pointData, std::vector<std::string>& pointDataNames);
//=============================================================================
// DATA
//=============================================================================

	
private:
	Model* _model;
	Model _MODEL;

	Array<double> _time;
	int _n_frames;
	int _n_out_frames;

	SimTK::Matrix _q_matrix;
	SimTK::Matrix _u_matrix;

	std::vector<std::string> _contact_force_names;
	std::vector<std::string> _contact_force_paths;
	std::vector<std::string> _contact_mesh_names;
	std::vector<std::string> _contact_mesh_paths;
	
    std::vector<std::string> _contact_output_double_names;
    std::vector<std::string> _contact_output_vec3_names;
    std::vector<std::string> _contact_output_vector_double_names;
    std::vector<SimTK::Matrix> _contact_output_double_values;
    std::vector<SimTK::Matrix_<SimTK::Vec3>> _contact_output_vec3_values;
    std::vector<std::vector<SimTK::Matrix>> _contact_output_vector_double_values;
    


    std::vector<std::string> _attach_geo_names;
	std::vector<std::string> _attach_geo_frames;
	std::vector<SimTK::Matrix_<SimTK::Vec3>> _mesh_vertex_locations;
	std::vector<SimTK::Matrix_<SimTK::Vec3>> _attach_geo_vertex_locations;
	std::vector<SimTK::PolygonalMesh> _attach_geo_meshes;
    
    int _max_path_points;

    std::vector<std::string> _ligament_names;
	std::vector<SimTK::Vector> _ligament_path_nPoints;
    std::vector < SimTK::Matrix_< SimTK::Vec3>> _ligament_path_points;
    std::vector<std::string> _ligament_output_double_names;
    std::vector<SimTK::Matrix> _ligament_output_double_values;

    std::vector<std::string> _muscle_names;
	std::vector<SimTK::Vector> _muscle_path_nPoints;
    std::vector < SimTK::Matrix_< SimTK::Vec3>> _muscle_path_points;
    std::vector<std::string> _muscle_output_double_names;
    std::vector<SimTK::Matrix> _muscle_output_double_values;

    std::vector<std::vector<std::string>> _muscle_state_names;
    std::vector<std::vector<SimTK::Vector>> _muscle_state_data;

    std::vector<std::string> _coordinate_names;
    std::vector<std::string> _coordinate_output_double_names;
    std::vector<SimTK::Matrix> _coordinate_output_double_values;

    TimeSeriesTable _frame_transform_in_ground;
    int _frame_transform_n_col;

    std::string _directoryOfSetupFile;
//=============================================================================
};  // END of class JointMechanicsTool

}; //namespace



#endif // #ifndef  OPENSIM_JOINT_MECHANICS_TOOL_H_
