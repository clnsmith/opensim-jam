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

#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim\Simulation\Model\Model.h>
#include "Smith2018ArticularContactForce.h"
#include "H5FileAdapter.h"
#include "osimPluginDLL.h"
#include "H5Cpp.h"
#include "hdf5_hl.h"
#include "OpenSim/Simulation/StatesTrajectory.h"


namespace OpenSim { 

//=============================================================================
//                        Joint Mechanics Tool
//=============================================================================
/**
The JointMechanicsTool enables detailed analyses of simulation results and the 
generation of output files for visualization outside the OpenSim GUI and 
further analyses in MATLAB/Python. To use this tool, a simulation must be 
performed (using a different tool) with the model states written to a .sto file.
The JointMechanicsTool reposes the model at the states listed in the 
states_file and performs further analyses. 

A key feature of the JointMechanicsTool is the ability to write .vtp files 
for each timestep that contain both geometric information (vertex locations 
and connectivity) and simulations outputs. These .vtp 
files can then be read into a visualization software such as Paraview to 
generate high quality renderings of simulation results. This feature is 
especially useful for models with Smith2018ArticularContact Components because 
it enables triangle level outputs (proximity, pressure, potential energy, etc).
to visualized on the mesh geometry. Muscle and Blankevoort1991Ligament 
components can also be written as .vtp files such that they can be colored
by their output values in visualizations. Finally, AttachedGeometry meshes 
(ie bones) can also be written as .vtp files for visualiation. The model
components and respective outputs that are written to .vtp files can be
controlled using the contacts, contact_outputs, ligaments, ligament_ouputs, 
muscles, muscle_outputs, and attached_geometry_bodies properties. 




*/
class OSIMPLUGIN_API JointMechanicsTool : public Object {

    OpenSim_DECLARE_CONCRETE_OBJECT(JointMechanicsTool, Object);

//=============================================================================
// PROPERTIES
//=============================================================================
public:
    OpenSim_DECLARE_PROPERTY(model_file, std::string,
        "Path to .osim file to use in analysis.")

    OpenSim_DECLARE_PROPERTY(states_file, std::string,
        "Path to storage file (.sto) containing the model states vs time for "
        "the simulation to be analyzed.")

    OpenSim_DECLARE_PROPERTY(results_directory, std::string,
        "Path to folder where the results files will be written.")

    OpenSim_DECLARE_PROPERTY(results_file_basename, std::string,
        "Prefix to each results file name.")

    OpenSim_DECLARE_PROPERTY(start_time, double,
        "Time to start the analysis. Set to -1 to use initial frame in "
        "states_file. The default value is -1.")

    OpenSim_DECLARE_PROPERTY(stop_time, double,
        "Time to stop the analysis. Set to -1 to use last frame in "
        "states_file. The default value is -1.")

    OpenSim_DECLARE_PROPERTY(resample_step_size, double,
        "Time step size to report results, set to -1 to use steps directly "
        "from the states_file. The default value is -1.")

    OpenSim_DECLARE_PROPERTY(normalize_to_cycle, bool,
        "Resample to 101 equally spaced time steps (percentage of activity "
        "cycle). Note: If true, this overrides resample_step_size. "
        "The default value is false.")

    OpenSim_DECLARE_PROPERTY(lowpass_filter_frequency, double,
        "Apply IIR lowpass butterworth filter to the input Coordinate values. "
        "If set to -1, no filtering is applied. The default value is -1.")

    OpenSim_DECLARE_PROPERTY(print_processed_kinematcs, bool,
        "Print a .sto file with the processed (cut, resampled, normalized, "
        "and filtered) kinematics used for posing the model through out the "
        "analysis.")

    OpenSim_DECLARE_LIST_PROPERTY(contacts, std::string, 
        "Paths to Smith2018ArticularContactForce components to be recorded. "
        "Options: 'none','all', or a list of Smith2018ArticularContactForce "
        "component paths. The default value is 'all'.")

    OpenSim_DECLARE_LIST_PROPERTY(contact_outputs, std::string, 
        "Names of Smith2018ArticularContactForce outputs that will "
        "be written to the results files. "
        "Options: 'none','all', or list of individual output names."
        "The default value is 'all'.")

    OpenSim_DECLARE_LIST_PROPERTY(contact_mesh_properties, std::string,
        "Names of Smith2018ArticularContactForce properties whose value will "
        "be written for each triangle in the mesh to the results files. "
        "Options: 'none','thickness','elastic modulus','poisson ratio',"
        "'area' or 'all'. "
        "The default value is 'none'.")

    OpenSim_DECLARE_LIST_PROPERTY(ligaments, std::string, 
        "Paths of Blankevoort1991Ligament components to be recorded. "
        "Options: 'none','all', or a list of Blankevoort1991Ligament "
        "component paths. The default value is 'none'.")

    OpenSim_DECLARE_LIST_PROPERTY(ligament_outputs, std::string, 
        "Names of Blankevoort1991Ligament outputs that will "
        "be written to the results files. "
        "Options: 'none','all', or list of individual output names. "
        "The default value is 'all'.")

    OpenSim_DECLARE_LIST_PROPERTY(muscles, std::string, 
        "Paths to Muscle components to be recorded. "
        "Options: 'none','all', or a list of Muscle component paths. "
        "The default value is 'none'.")

    OpenSim_DECLARE_LIST_PROPERTY(muscle_outputs, std::string, 
        "Names of Muscle component outputs that will "
        "be written to the results files. "
        "Options: 'none','all', or list of individual output names. "
        "The default value is 'all'.")

    OpenSim_DECLARE_LIST_PROPERTY(attached_geometry_bodies, std::string, 
        "Paths to the Body components that contain attached geometry Mesh "
        "components (i.e. bone meshes) to be recorded. "
        "Options: 'none','all', or a list of Frame component paths. "
        "The default value is 'none'.")

    OpenSim_DECLARE_PROPERTY(output_origin, std::string,
        "Body or Frame to use as origin for model motion in results files. "
        "Options: 'ground' or '/path/to/Frame")

    OpenSim_DECLARE_PROPERTY(output_frame, std::string,
        "Body or Frame to use as reference frame for model motion in results "
        "files. "
        "Options: 'ground' or '/path/to/Frame' ")

    OpenSim_DECLARE_PROPERTY(write_vtp_files, bool,
        "Write .vtp files for visualization. The default value is true.")

    OpenSim_DECLARE_PROPERTY(write_h5_file, bool,
        "Write binary .h5 file")

    OpenSim_DECLARE_PROPERTY(h5_states_data, bool,
        "Write states data to .h5 file")

    OpenSim_DECLARE_PROPERTY(h5_kinematics_data, bool,
        "Write kinematics data to .h5 file")

    OpenSim_DECLARE_UNNAMED_PROPERTY(AnalysisSet,"Analyses to be performed "
        "during forward simulation.")


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
    
    void initialize(SimTK::State& state);
    void formQandUMatrixFromFile();
    
    int record(const SimTK::State& s, const int frame_num);
    
    
    void writeVTPFile(const std::string& mesh_name,
        const std::vector<std::string>& contact_names, bool isDynamic);

    void writeAttachedGeometryVTPFiles(bool isDynamic);

    void writeLineVTPFiles(std::string line_name,
        const SimTK::Vector& nPoints, const SimTK::Matrix_<SimTK::Vec3>& path_points,
        const std::vector<std::string>& output_double_names, const SimTK::Matrix& output_double_values);
    void writeH5File(const std::string &aBaseName, const std::string &aDir);

    void setupLigamentStorage();
    void setupMuscleStorage();
    void setupAttachedGeometriesStorage();
    void setupCoordinateStorage();
    void setupContactStorage(SimTK::State& state);
    std::string findMeshFile(const std::string& file);

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
    std::vector<SimTK::Matrix_<SimTK::Vec3>> _mesh_vertex_locations;
    
    std::vector<std::string> _contact_output_double_names;
    std::vector<std::string> _contact_output_vec3_names;
    std::vector<std::string> _contact_output_vector_double_names;
    std::vector<SimTK::Matrix> _contact_output_double_values;
    std::vector<SimTK::Matrix_<SimTK::Vec3>> _contact_output_vec3_values;
    std::vector<std::vector<SimTK::Matrix>> _contact_output_vector_double_values;

    std::vector<std::string> _attach_geo_names;
    std::vector<std::string> _attach_geo_frames;
    std::vector<SimTK::PolygonalMesh> _attach_geo_meshes;
    std::vector<SimTK::Matrix_<SimTK::Vec3>> _attach_geo_vertex_locations;
    
    int _max_path_points;

    std::vector<std::string> _ligament_names;
    std::vector<std::string> _ligament_paths;
    std::vector<SimTK::Vector> _ligament_path_nPoints;
    std::vector < SimTK::Matrix_< SimTK::Vec3>> _ligament_path_points;
    std::vector<std::string> _ligament_output_double_names;
    std::vector<SimTK::Matrix> _ligament_output_double_values;

    std::vector<std::string> _muscle_names;
    std::vector<std::string> _muscle_paths;
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
