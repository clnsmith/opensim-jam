/* -------------------------------------------------------------------------- *
 *                          JointMechanicsTool.cpp                            *
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
#include "JointMechanicsTool.h"
#include "VTPFileAdapter.h"
#include "H5Cpp.h"
#include "hdf5_hl.h"
#include "Smith2018ArticularContactForce.h"
#include "HelperFunctions.h"
#include "Blankevoort1991Ligament.h"
#include <OpenSim/Analyses/StatesReporter.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/CSVFileAdapter.h>

using namespace OpenSim;


//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================

JointMechanicsTool::JointMechanicsTool() : Object()
{
	setNull();
	constructProperties();
    _directoryOfSetupFile = "";

}


JointMechanicsTool::JointMechanicsTool(std::string settings_file) : 
    Object(settings_file) {
	constructProperties();
	updateFromXMLDocument();
	
	loadModel(settings_file);
	_directoryOfSetupFile = IO::getParentDirectory(settings_file);
    IO::chDir(_directoryOfSetupFile);    
}

JointMechanicsTool::JointMechanicsTool(Model *aModel, 
	std::string coordinates_file, std::string results_dir) :
	JointMechanicsTool()
{
    if(aModel==NULL) return;
    setModel(*aModel);

	set_coordinates_file(coordinates_file);
	set_results_directory(results_dir);


}

void JointMechanicsTool::setNull()
{
    setAuthors("Colin Smith");
}

void JointMechanicsTool::constructProperties()
{
    Array<std::string> defaultListNames;
	defaultListNames.append("all");

	Array<std::string> defaultAttachGeo;
	defaultAttachGeo.append("none");

	constructProperty_model_file("");
	constructProperty_coordinates_file("");
	constructProperty_results_directory(".");
	constructProperty_results_file_basename("");
	constructProperty_start_time(-1);
	constructProperty_stop_time(-1);
	constructProperty_resample_step_size(-1);
	constructProperty_normalize_to_cycle(false);
	constructProperty_smoothing_spline_frequency(-1);
	constructProperty_smoothing_spline_order(5);
	constructProperty_lowpass_filter_frequency(-1);
	constructProperty_print_processed_kinematcs(false);
	constructProperty_contact_names(defaultListNames);
	constructProperty_tri_output_data_names(defaultListNames);
    
    constructProperty_write_vtp_files(true);
    constructProperty_vtp_origin("ground");
    constructProperty_vtp_frame("ground");
    constructProperty_vtp_contact(true);
	constructProperty_vtp_attached_geometries(defaultAttachGeo);
	constructProperty_vtp_ligaments(true);
    constructProperty_vtp_muscles(true);	
	constructProperty_vtp_contact_properties(defaultListNames);

    constructProperty_write_h5_file(true);
	constructProperty_h5_raw_contact_data(true);
	constructProperty_h5_summary_contact_data(true);
    constructProperty_h5_regional_summary_contact(true);
	constructProperty_h5_states_data(true);
	constructProperty_h5_kinematics_data(true);
	constructProperty_h5_muscle_data(true);
	constructProperty_h5_ligament_data(true);
}

void JointMechanicsTool::setModel(Model& aModel)
{
	_model = &aModel;
	set_model_file(_model->getDocumentFileName());
}


void JointMechanicsTool::run() {
	_max_path_points = 100;

	if (_model == NULL) {
		OPENSIM_THROW(Exception, "No model was set in JointMechanicsTool");
	}
	
	_model->initSystem();	
    
	//Get Coordinates and Speeds from file
	formQandUMatrixFromFile();
	
	initializeSettings();
	
	SimTK::State state = _model->initSystem();
	
    
	setModelingOptions(state);
	
	AnalysisSet& analysisSet = _model->updAnalysisSet();

	//loop over each frame
	for (int i = 0; i < _n_frames; ++i) {
		
		//Set Time
		state.setTime(_time[i]);

		std::cout << "Time: " << _time[i] << std::endl;

		//Set Qs and Us
		int nCoord = 0;
		for (const Coordinate& coord : _model->getComponentList<Coordinate>()) {
            coord.setValue(state, _q_matrix(i, nCoord));
            coord.setSpeedValue(state, _u_matrix(i, nCoord));
			nCoord++;
		}

        //Set Muscle States
        if (get_vtp_muscles()) {
            int nMsl = 0;
            for (const Muscle& msl : _model->getComponentList<Muscle>()) {
                for (int j = 0; j < _muscle_state_names[nMsl].size(); ++j) {
                    msl.setStateVariableValue(state, _muscle_state_names[nMsl][j], _muscle_state_data[nMsl][j][i]);
                }
                nMsl++;
            }
        }
        //Record Values
		record(state,i);

		//Perform analyses
		if (i == 0) {
			analysisSet.begin(state);
		}
		else {
			analysisSet.step(state, i);
		}
	}
	printResults(get_results_file_basename(), get_results_directory());
}

void JointMechanicsTool::formQandUMatrixFromFile() {

    //load coordinates file
    //Storage store;
    //if (_directoryOfSetupFile != "") {
        std::string saveWorkingDirectory = IO::getCwd();
        IO::chDir(_directoryOfSetupFile);

        //try {
            Storage store = Storage(get_coordinates_file());

        //}
        //catch (...) { // Properly restore current directory if an exception is thrown
        //    IO::chDir(saveWorkingDirectory);
        //    throw;
        //}
        IO::chDir(saveWorkingDirectory);
    //}
    //else{
    //    store = Storage(get_coordinates_file());
    //}
	//Set Start and Stop Times
	store.getTimeColumn(_time);
	 
	if (get_start_time() == -1) {
		set_start_time(_time.get(0));
	}
	if (get_stop_time() == -1) {
		set_stop_time(_time.getLast());
	}

	const CoordinateSet& coordinateSet = _model->getCoordinateSet();
	
	if (store.isInDegrees()) {
		_model->getSimbodyEngine().convertDegreesToRadians(store);
    }

    if (get_smoothing_spline_frequency() != -1) {
        store.pad(store.getSize() / 2);
        store.smoothSpline(get_smoothing_spline_order(), get_smoothing_spline_frequency());
    }
    else if (get_lowpass_filter_frequency() != -1) {
        store.pad(store.getSize() / 2);
        store.lowpassIIR(get_lowpass_filter_frequency());
    }

    //Cut to start and stop times
    store.crop(get_start_time(), get_stop_time());

    if (get_normalize_to_cycle() == true) {
        double norm_dt = (get_stop_time() - get_start_time()) / 100;
        store.resampleLinear(norm_dt);
    }
    else if (get_resample_step_size() != -1 && get_normalize_to_cycle() == false) {
        store.resampleLinear(get_resample_step_size());
    }

    if (get_print_processed_kinematcs()) {
        store.print(get_results_directory() + "/" + get_results_file_basename() + "_processed_kinematics.sto");
    }

    //Update the time 
    store.getTimeColumn(_time);

    //Set number of Frames
    _n_frames = _time.size();

    //Gather Q and U values
    Array<std::string> col_labels = store.getColumnLabels();

    Array<int> q_col_map(-1, _model->getNumCoordinates());


    for (int i = 0; i < col_labels.size(); ++i) {
        std::vector<std::string> split_label = split_string(col_labels[i], "/");

        int j = 0;
        for (const Coordinate& coord : _model->getComponentList<Coordinate>()) {
            if (contains_string(split_label, coord.getName())) {
                if (split_label.back() == "value" || split_label.back() == coord.getName()) {
                    q_col_map[j] = i;
                }
            }
            j++;
        }

    }

    _q_matrix.resize(_n_frames, _model->getNumCoordinates());
    _u_matrix.resize(_n_frames, _model->getNumCoordinates());

    _q_matrix = 0;
    _u_matrix = 0;

    int j = 0;
    for (const Coordinate& coord : _model->getComponentList<Coordinate>()) {

        if (q_col_map[j] != -1) {
            double* data = NULL;
            store.getDataColumn(col_labels[q_col_map[j]], data);
            for (int i = 0; i < _n_frames; ++i) {
                _q_matrix(i, j) = data[i];
            }
        }
        else {
            std::cout << "Coordinate Value: " << coord.getName() << " not found in coordinates_file, assuming 0." << std::endl;
        }

        GCVSpline q_spline;
        q_spline.setDegree(5);

        for (int i = 0; i < _n_frames; ++i) {       
            q_spline.addPoint(_time[i], _q_matrix(i,j));
        }

        for (int i = 0; i < _n_frames; ++i) {
            SimTK::Vector x(1);
            x(0) = _time[i];

            std::vector<int> u_order = { 0 };
            _u_matrix(i, j) = q_spline.calcDerivative(u_order, x);
        }
        j++;
    }

    //Gather Muscle States
    if (get_vtp_muscles()) {
        for (const Muscle& msl : _model->updComponentList<Muscle>()) {
            std::vector<std::string> state_names;
            std::vector<SimTK::Vector> state_values;

            Array<std::string> stateVariableNames = msl.getStateVariableNames();

            for (int i = 0; i < stateVariableNames.getSize(); ++i) {
                state_names.push_back(stateVariableNames[i]);
            }
            _muscle_state_names.push_back(state_names);

            for (std::string msl_state : state_names) {
                int col_ind = -1;
                for (int j = 0; j < col_labels.size(); ++j) {
                    if (col_labels[j] == msl_state) {
                        col_ind = j;
                        break;
                    }
                }

                SimTK::Vector state_data(_n_frames, 0.0);
                if (col_ind == -1) {
                    std::cout << "WARNING:: Muscle state (" + msl_state + ") NOT found in coordinates file. Assumed 0.0" << std::endl;
                }
                else {
                    Array<double> data;
                    store.getDataColumn(col_labels[col_ind], data);                    
                    for (int j = 0; j < data.getSize(); ++j) {
                        state_data.set(j, data[j]);
                    }
                }
                state_values.push_back(state_data);
            }
            _muscle_state_data.push_back(state_values);
        }
    }
}

void JointMechanicsTool::initializeSettings() {
    //Check Contact Names
	if (get_contact_names(0) == "all") {
		for (const Smith2018ArticularContactForce& contactForce : _model->getComponentList<Smith2018ArticularContactForce>()) {
			_contact_force_names.push_back(contactForce.getName());
			_contact_force_paths.push_back(contactForce.getAbsolutePathString());

			std::string casting_mesh_name = contactForce.getConnectee<Smith2018ContactMesh>("casting_mesh").getName();
			std::string target_mesh_name = contactForce.getConnectee<Smith2018ContactMesh>("target_mesh").getName();
			std::string casting_mesh_path = contactForce.getConnectee<Smith2018ContactMesh>("casting_mesh").getAbsolutePathString();
			std::string target_mesh_path = contactForce.getConnectee<Smith2018ContactMesh>("target_mesh").getAbsolutePathString();

			if (!contains_string(_contact_mesh_names, casting_mesh_name)) {
				_contact_mesh_names.push_back(casting_mesh_name);
				_contact_mesh_paths.push_back(casting_mesh_name);
			}
			if (!contains_string(_contact_mesh_names, target_mesh_name)) {
				_contact_mesh_names.push_back(target_mesh_name);
				_contact_mesh_paths.push_back(target_mesh_name);
			}
		}
	}
	else {
		for (int i = 0; i < getProperty_contact_names().size(); ++i) {
			try {
				const Smith2018ArticularContactForce& contactForce = _model->getComponent<Smith2018ArticularContactForce>(get_contact_names(i));
				_contact_force_names.push_back(contactForce.getName());
				_contact_force_paths.push_back(contactForce.getAbsolutePathString());

				std::string casting_mesh_name = contactForce.getConnectee<Smith2018ContactMesh>("casting_mesh").getName();
				std::string target_mesh_name = contactForce.getConnectee<Smith2018ContactMesh>("target_mesh").getName();
				std::string casting_mesh_path = contactForce.getConnectee<Smith2018ContactMesh>("casting_mesh").getAbsolutePathString();
				std::string target_mesh_path = contactForce.getConnectee<Smith2018ContactMesh>("target_mesh").getAbsolutePathString();

				if (!contains_string(_contact_mesh_names, casting_mesh_name)) {
					_contact_mesh_names.push_back(casting_mesh_name);
					_contact_mesh_paths.push_back(casting_mesh_path);
				}
				if (!contains_string(_contact_mesh_names, target_mesh_name)) {
					_contact_mesh_names.push_back(target_mesh_name);
					_contact_mesh_paths.push_back(target_mesh_path);
				}
			}
			catch (ComponentNotFoundOnSpecifiedPath){
				OPENSIM_THROW(Exception, "contact_name: " + get_contact_names(i)
					+ " was not found as a Smith2018ArticularContactForce path" 
					" in the model. Did you use absolute path?");
			}
		}
	}
	
	//States	
	if (get_h5_states_data()) {
		StatesReporter* states_rep = new StatesReporter();
		states_rep->setName("states_analysis");
		states_rep->setStepInterval(1);
		states_rep->setPrintResultFiles(false);
		_model->addAnalysis(states_rep);
	}
	

}
void JointMechanicsTool::setModelingOptions(SimTK::State& state) {

	for (int i = 0; i < _contact_force_paths.size(); ++i) {

		Smith2018ArticularContactForce& contactForce = _model->updComponent
			<Smith2018ArticularContactForce>(_contact_force_paths[i]);

		contactForce.setModelingOption(state, "flip_meshes", 1);		
	}
    
	setupDynamicVertexLocationStorage();

	if (get_vtp_ligaments() || get_h5_ligament_data()) {
		setupLigamentStorage();
	}
    if (get_vtp_muscles() || get_h5_muscle_data()) {
		setupMuscleStorage();
	}
    if (get_h5_kinematics_data()) {
        setupCoordinateStorage();
    }
    if (get_vtp_contact() || get_h5_raw_contact_data() || get_h5_summary_contact_data()) {
        _model->realizeReport(state);
        setupContactStorage(state);
    }
}

                                                                
void JointMechanicsTool::setupDynamicVertexLocationStorage() {
    std::string model_file = SimTK::Pathname::getAbsolutePathname(_model->getDocumentFileName());
    std::string model_dir, dummy1, dummy2;
    bool dummyBool;
    SimTK::Pathname::deconstructPathname(model_file, dummyBool, model_dir, dummy1, dummy2);
    
    
    if (get_write_vtp_files()) {
        if (get_vtp_frame() != "static") {
            _mesh_vertex_locations.resize(_contact_mesh_names.size());

            for (int i = 0; i < _contact_mesh_names.size(); ++i) {

                int mesh_nVer = _model->getComponent<Smith2018ContactMesh>
                    (_contact_mesh_names[i]).getPolygonalMesh().getNumVertices();

                _mesh_vertex_locations[i].resize(_n_frames, mesh_nVer);
            }
        }

        if (get_vtp_attached_geometries(0) == "all") {
            for (Frame& frame : _model->updComponentList<Frame>()) {
                for (int i = 0; i < frame.getProperty_attached_geometry().size(); ++i) {
                    const Geometry& geo = frame.get_attached_geometry(i);

                    if (geo.getConcreteClassName() != "Mesh") {
                        continue;
                    }

                    if (contains_string(_attach_geo_names, geo.getName())) {
                        continue;
                    }



                    Mesh* mesh = (Mesh*)&geo;
                    std::string filename = mesh->get_mesh_file();
                    std::ifstream file(filename);
                    if (!file) {
                        filename = model_dir + mesh->get_mesh_file();
                        file.open(filename);
                    }
                    if (!file) {
                        filename = model_dir + "Geometry/" + mesh->get_mesh_file();
                        file.open(filename);
                    }

                    if (!file) {
                        OPENSIM_THROW(Exception, "Attached Geometry file doesn't exist:\n" + model_dir + "[Geometry/]" + mesh->get_mesh_file());
                    }

                    SimTK::PolygonalMesh ply_mesh;
                    ply_mesh.loadFile(filename);

                    //Apply Scale Factors
                    SimTK::Vec3 scale = mesh->get_scale_factors();
                    if (scale != SimTK::Vec3(1)) {
                        SimTK::PolygonalMesh scaled_mesh;

                        for (int v = 0; v < ply_mesh.getNumVertices(); ++v) {
                            scaled_mesh.addVertex(ply_mesh.getVertexPosition(v).elementwiseMultiply(scale));
                        }

                        for (int f = 0; f < ply_mesh.getNumFaces(); ++f) {
                            SimTK::Array_<int> facevertex;
                            for (int k = 0; k < ply_mesh.getNumVerticesForFace(f); ++k) {
                                facevertex.push_back(ply_mesh.getFaceVertex(f, k));
                            }
                            scaled_mesh.addFace(facevertex);
                        }
                        ply_mesh.copyAssign(scaled_mesh);
                    }

                    _attach_geo_names.push_back(geo.getName());
                    _attach_geo_frames.push_back(frame.getAbsolutePathString());
                    _attach_geo_meshes.push_back(ply_mesh);
                    _attach_geo_vertex_locations.push_back(SimTK::Matrix_<SimTK::Vec3>(_n_frames, ply_mesh.getNumVertices()));
                }
            }
        }
        else if (get_vtp_attached_geometries(0) != "none") {
            
            for (int j = 0; j < getProperty_vtp_attached_geometries().size(); ++j) {
                Frame& frame = _model->updComponent<Frame>(get_vtp_attached_geometries(j));
                for (int i = 0; i < frame.getProperty_attached_geometry().size(); ++i) {
                    const Geometry& geo = frame.get_attached_geometry(i);

                    if (geo.getConcreteClassName() != "Mesh") {
                        continue;
                    }

                    if (contains_string(_attach_geo_names, geo.getName())) {
                        continue;
                    }

                    Mesh* mesh = (Mesh*)&geo;
                    std::string filename = mesh->get_mesh_file();
                    std::ifstream file(filename);
                    if (!file) {
                        filename = model_dir + mesh->get_mesh_file();
                        file.open(filename);
                    }
                    if (!file) {
                        filename = model_dir + "Geometry/" + mesh->get_mesh_file();
                        file.open(filename);
                    }

                    if (!file) {
                        OPENSIM_THROW(Exception, "Attached Geometry file doesn't exist:\n" + model_dir + "[Geometry/]" + mesh->get_mesh_file());
                    }

                    SimTK::PolygonalMesh ply_mesh;
                    ply_mesh.loadFile(filename);

                    //Apply Scale Factors
                    SimTK::Vec3 scale = mesh->get_scale_factors();
                    if (scale != SimTK::Vec3(1)) {
                        SimTK::PolygonalMesh scaled_mesh;

                        for (int v = 0; v < ply_mesh.getNumVertices(); ++v) {
                            scaled_mesh.addVertex
                            (ply_mesh.getVertexPosition(v).elementwiseMultiply(scale));
                        }

                        for (int f = 0; f < ply_mesh.getNumFaces(); ++f) {
                            SimTK::Array_<int> facevertex;
                            for (int k = 0; k < ply_mesh.getNumVerticesForFace(f); ++k) {
                                facevertex.push_back(ply_mesh.getFaceVertex(f, k));
                            }
                            scaled_mesh.addFace(facevertex);
                        }
                        ply_mesh.copyAssign(scaled_mesh);
                    }

                    _attach_geo_names.push_back(geo.getName());
                    _attach_geo_frames.push_back(frame.getAbsolutePathString());
                    _attach_geo_meshes.push_back(ply_mesh);
                    _attach_geo_vertex_locations.push_back(SimTK::Matrix_<SimTK::Vec3>(_n_frames, ply_mesh.getNumVertices()));
                }
            }
        }
    }
}

void JointMechanicsTool::setupLigamentStorage() {
    if (_model->countNumComponents<Blankevoort1991Ligament>() == 0) return;

    const Blankevoort1991Ligament& lig0 = _model->updComponentList<Blankevoort1991Ligament>().begin().deref();

    for (const auto& entry : lig0.getOutputs()) {
        const AbstractOutput* output = entry.second.get();
        if(output->isListOutput()){continue;}
        if (output->getTypeName() == "double") {            
            _ligament_output_double_names.push_back(output->getName());
        }
    }
    int nLigamentOutputs = _ligament_output_double_names.size();   
    SimTK::Matrix lig_output_data(_n_frames, nLigamentOutputs,-1);

    for (const Blankevoort1991Ligament& lig : _model->getComponentList<Blankevoort1991Ligament>()) {
        //Path Storage
        SimTK::Matrix_<SimTK::Vec3>	lig_matrix(_n_frames, _max_path_points, SimTK::Vec3(-1));
        SimTK::Vector lig_vector(_n_frames, -1);
        _ligament_names.push_back(lig.getName());
        _ligament_path_points.push_back(lig_matrix);
        _ligament_path_nPoints.push_back(lig_vector);
        
        //Data Storage
        _ligament_output_double_values.push_back(lig_output_data);
    }
}

void JointMechanicsTool::setupMuscleStorage() {
    if (_model->countNumComponents<Muscle>() == 0) return;
    const Muscle& msl0 = _model->getMuscles().get(0);
    
    for (const auto& entry : msl0.getOutputs()) {
        const AbstractOutput* output = entry.second.get();
        if(output->isListOutput()){continue;}
        if (output->getTypeName() == "double") {
            _muscle_output_double_names.push_back(output->getName());
        }
    }
    int nMuscleOutputs = _muscle_output_double_names.size();   
    SimTK::Matrix msl_output_data(_n_frames, nMuscleOutputs,-1);

    for (const Muscle& msl : _model->updComponentList<Muscle>()) {
        
        //Path Storage
        SimTK::Matrix_<SimTK::Vec3>	msl_matrix(_n_frames, _max_path_points, SimTK::Vec3(-1));
        SimTK::Vector msl_vector(_n_frames, -1);
        _muscle_names.push_back(msl.getName());
        _muscle_path_points.push_back(msl_matrix);
        _muscle_path_nPoints.push_back(msl_vector);

        //Data Storage
        _muscle_output_double_values.push_back(msl_output_data);
    }

    
}

void JointMechanicsTool::setupCoordinateStorage() {
    _coordinate_output_double_names.push_back("value");
    _coordinate_output_double_names.push_back("speed");

    for (const Coordinate& coord : _model->updComponentList<Coordinate>()) {
        _coordinate_names.push_back(coord.getName());

        SimTK::Matrix coord_data(_n_frames, 2, -1.0);
        _coordinate_output_double_values.push_back(coord_data);
    }
}

void JointMechanicsTool::setupContactStorage(const SimTK::State& state) {
     if (_model->countNumComponents<Smith2018ArticularContactForce>() == 0) return;

    const Smith2018ArticularContactForce& frc0 = _model->getComponent<Smith2018ArticularContactForce>(_contact_force_paths[0]);

    for (const auto& entry : frc0.getOutputs()) {
        const std::string& output_name = entry.first;
        const AbstractOutput* output = entry.second.get();
        
        if(output->isListOutput()){continue;}
                
        if (output->getTypeName() == "double") {
            _contact_output_double_names.push_back(output->getName());            
        }
        if (output->getTypeName() == "Vec3") {
            _contact_output_vec3_names.push_back(output->getName());
        }
        if (output->getTypeName() == "Vector") {
            _contact_output_vector_double_names.push_back(output->getName());
            
        }
    }

    int nOutputDouble = _contact_output_double_names.size();
    int nOutputVec3 = _contact_output_vec3_names.size();
    int nOutputVector = _contact_output_vector_double_names.size();

    SimTK::Matrix double_data(_n_frames, nOutputDouble,-1);
    SimTK::Matrix_<SimTK::Vec3> vec3_data(_n_frames, nOutputVec3,SimTK::Vec3(-1));    

    for (std::string frc_path : _contact_force_paths) {
        const Smith2018ArticularContactForce& frc = _model->updComponent<Smith2018ArticularContactForce>(frc_path);

        _contact_output_double_values.push_back(double_data);
        _contact_output_vec3_values.push_back(vec3_data);

        std::vector<SimTK::Matrix> def_output_vector;

        for (int i = 0; i < nOutputVector; ++i) {
            const AbstractOutput& abs_output = frc.getOutput(_contact_output_vector_double_names[i]);
            
            const Output<SimTK::Vector>& vector_output = dynamic_cast<const Output<SimTK::Vector>&>(abs_output);
            int output_vector_size = vector_output.getValue(state).size();
            
            def_output_vector.push_back(SimTK::Matrix(_n_frames, output_vector_size,-1));            
        }
        _contact_output_vector_double_values.push_back(def_output_vector);

    }
}

int JointMechanicsTool::record(const SimTK::State& s, const int frame_num)
{
    _model->realizeReport(s);

    //Store mesh vertex locations
	std::string frame_name = get_vtp_frame();
    const Frame& frame = _model->getComponent<Frame>(frame_name);
    std::string origin_name = get_vtp_origin();
    const Frame& origin = _model->getComponent<Frame>(origin_name);

    SimTK::Vec3 origin_pos = origin.findStationLocationInAnotherFrame(s, SimTK::Vec3(0), frame);

    if (get_write_vtp_files()) {
        if (get_vtp_frame() != "static") {
            for (int i = 0; i < _contact_mesh_paths.size(); ++i) {
                int nVertex = _mesh_vertex_locations[i].ncol();
                      

                SimTK::Vector_<SimTK::Vec3> ver = _model->getComponent<Smith2018ContactMesh>
                    (_contact_mesh_paths[i]).getVertexLocations(); 
                const SimTK::Transform& T = _model->getComponent<Smith2018ContactMesh>
                    (_contact_mesh_paths[i]).get_mesh_frame().findTransformBetween(s,frame);

                for (int j = 0; j < nVertex; ++j) {
                    _mesh_vertex_locations[i](frame_num, j) = T.shiftFrameStationToBase(ver(j));
                }
            }
        }


        if (get_vtp_attached_geometries(0) != "none") {
            for (int i = 0; i < _attach_geo_names.size(); ++i) {

                const SimTK::PolygonalMesh& mesh = _attach_geo_meshes[i];


                SimTK::Transform trans = _model->getComponent<PhysicalFrame>(_attach_geo_frames[i]).findTransformBetween(s, frame);

                for (int j = 0; j < mesh.getNumVertices(); ++j) {
                    _attach_geo_vertex_locations[i](frame_num, j) = trans.shiftFrameStationToBase(mesh.getVertexPosition(j)) - origin_pos;
                }
            }
        }
    }
    
    //Store Contact data
    if (get_vtp_contact() || get_h5_raw_contact_data()) {
        int nFrc = 0;
        for (std::string frc_path : _contact_force_paths) {
            const Smith2018ArticularContactForce& frc = _model->updComponent<Smith2018ArticularContactForce>(frc_path);

            int nDouble = 0;
            for (std::string output_name : _contact_output_double_names) {
                _contact_output_double_values[nFrc].set(frame_num, nDouble, frc.getOutputValue<double>(s, output_name));
                nDouble++;
            }

            int nVec3 = 0;
            for (std::string output_name : _contact_output_vec3_names) {
                _contact_output_vec3_values[nFrc].set(frame_num, nVec3, frc.getOutputValue<SimTK::Vec3>(s, output_name));
                nVec3++;
            }
            
            int nVector = 0;
            for (std::string output_name : _contact_output_vector_double_names) {                
                _contact_output_vector_double_values[nFrc][nVector].updRow(frame_num) = ~frc.getOutputValue<SimTK::Vector>(s, output_name);                                
                nVector++;
            }
            nFrc++;
        }

    }

    //Store ligament data
    if (get_vtp_ligaments() || get_h5_ligament_data()) {            
        int nLig = 0;
        for (Blankevoort1991Ligament& lig : _model->updComponentList<Blankevoort1991Ligament>()) {

            //Path Points
            const GeometryPath& geoPath = lig.upd_GeometryPath();

            int nPoints = 0;
            SimTK::Vector_<SimTK::Vec3> path_points(_max_path_points, SimTK::Vec3(-1));

            getGeometryPathPoints(s, geoPath, path_points, nPoints);
            for (int i = 0; i < nPoints; ++i) {
                _ligament_path_points[nLig].set(frame_num,i,path_points(i));
            }
            _ligament_path_nPoints[nLig][frame_num] = nPoints;
                
            //Output Data
            int j = 0;
            for (std::string output_name : _ligament_output_double_names) {
                _ligament_output_double_values[nLig].set(frame_num,j, lig.getOutputValue<double>(s, output_name));
                j++;
            }
            nLig++;
        }
    }

    //Store muscle data
    if (get_vtp_muscles() || get_h5_muscle_data()) {
        int nMsl = 0;
        for (Muscle& msl : _model->updComponentList<Muscle>()) {

            //Path Points
            const GeometryPath& geoPath = msl.upd_GeometryPath();

            int nPoints = 0;
            SimTK::Vector_<SimTK::Vec3> path_points(_max_path_points,SimTK::Vec3(-1));
            getGeometryPathPoints(s, geoPath, path_points, nPoints);
            for (int i = 0; i < nPoints; ++i) {
                _muscle_path_points[nMsl].set(frame_num,i,path_points(i));
            }
            _muscle_path_nPoints[nMsl][frame_num] = nPoints;
                

            //Output Data
            int j = 0;
            for (std::string output_name : _muscle_output_double_names) {
                _muscle_output_double_values[nMsl].set(frame_num,j, msl.getOutputValue<double>(s, output_name));
                j++;
            }
            nMsl++;
        }
    }
    
    //Store Coordinate Data
    if (get_h5_kinematics_data()) {
        int nCoord = 0;
        for (const Coordinate& coord : _model->updComponentList<Coordinate>()) {
            _coordinate_output_double_values[nCoord](frame_num,0) = coord.getValue(s);
            _coordinate_output_double_values[nCoord](frame_num,1) = coord.getSpeedValue(s);   
            nCoord++;
        }


    }
	return(0);
}

void JointMechanicsTool::getGeometryPathPoints(const SimTK::State& s, const GeometryPath& geoPath, SimTK::Vector_<SimTK::Vec3>& path_points, int& nPoints) {
    const Frame& out_frame = _model->getComponent<Frame>(get_vtp_frame());
    
    const Frame& origin = _model->getComponent<Frame>(get_vtp_origin());

    SimTK::Vec3 origin_pos = origin.findStationLocationInAnotherFrame(s, SimTK::Vec3(0), out_frame);

    const Array<AbstractPathPoint*>& pathPoints = geoPath.getCurrentPath(s);
    
    nPoints = 0;
    for (int i = 0; i < pathPoints.getSize(); ++i) {
        AbstractPathPoint* point = pathPoints[i];
        PathWrapPoint* pwp = dynamic_cast<PathWrapPoint*>(point);
        SimTK::Vec3 pos;

        //If wrapping point, need to collect all points on wrap object surface
        if (pwp) {
            Array<SimTK::Vec3>& surfacePoints = pwp->getWrapPath();
            const SimTK::Transform& X_BG = pwp->getParentFrame().findTransformBetween(s, out_frame);
            // Cycle through each surface point and tranform to output frame
            for (int j = 0; j < surfacePoints.getSize(); ++j) {
                pos = X_BG * surfacePoints[j]-origin_pos;
                path_points.set(nPoints, pos);
                nPoints++;                
            }
        }
        else { // otherwise a regular PathPoint so just draw its location
            const SimTK::Transform& X_BG = point->getParentFrame().findTransformBetween(s, out_frame);
            pos = X_BG * point->getLocation(s)-origin_pos;

            path_points.set(nPoints, pos);
            nPoints++;            
        }        
    }

    
}

//=============================================================================
// IO
//=============================================================================
//_____________________________________________________________________________
/**
 * Print results.
 *
 * The file names are constructed as
 * aDir + "/" + aBaseName + "_" + ComponentName + aExtension
 *
 * @param aDir Directory in which the results reside.
 * @param aBaseName Base file name.
 * @param aDT Desired time interval between adjacent storage vectors.  Linear
 * interpolation is used to print the data out at the desired interval.
 * @param aExtension File extension.
 *
 * @return 0 on success, -1 on error.
 */
int JointMechanicsTool::printResults(const std::string &aBaseName,const std::string &aDir)
{
    //Write VTP files
    if (get_write_vtp_files()) {
        //Contact Meshes
        for (std::string mesh_name : _contact_mesh_names) {
            if (get_vtp_frame() == "static") {
                writeVTPFile(mesh_name, _contact_force_names, false);
            }
            else{
                writeVTPFile(mesh_name, _contact_force_names, true);
            }
        }

        //Bones
        if (get_vtp_attached_geometries(0) != "none") {
            if (get_vtp_frame() == "static") {
                writeAttachedGeometryVTPFiles(false);
            }
            else {
                writeAttachedGeometryVTPFiles(true);
            }
        }

        //Ligaments
        if (get_vtp_ligaments()) {
            int i = 0;
            for (std::string lig : _ligament_names) {
                writeLineVTPFiles("ligament_" + lig, _ligament_path_nPoints[i],
                    _ligament_path_points[i], _ligament_output_double_names,
                    _ligament_output_double_values[i]);
                i++;
            }
        }

        //Muscles
        if (get_vtp_muscles()) {
            int i = 0;
            for (std::string msl : _muscle_names) {
                writeLineVTPFiles("muscle_" + msl, _muscle_path_nPoints[i],
                    _muscle_path_points[i], _muscle_output_double_names,
                    _muscle_output_double_values[i]);
                i++;
            }
        }
    }

	//Write h5 file
	if (get_write_h5_file()) {
		writeH5File(aBaseName, aDir);
	}

    return(0);
}

void JointMechanicsTool::writeBlenderCSVFile() {
    std::string csv_file = get_results_directory() + "/" + get_results_file_basename() + "_blender.csv";
    CSVFileAdapter csv;
    csv.write(_frame_transform_in_ground, csv_file);
}


void JointMechanicsTool::collectMeshContactOutputData(const std::string& mesh_name,
    std::vector<SimTK::Matrix>& triData, std::vector<std::string>& triDataNames,
    std::vector<SimTK::Matrix>& vertexData, std::vector<std::string>& vertexDataNames) {
    
    Smith2018ContactMesh mesh;
    
    int nFrc = -1;
    for (std::string frc_path : _contact_force_paths) {
        nFrc++;
        
        std::string mesh_type = "";
        const Smith2018ArticularContactForce& frc = _model->updComponent<Smith2018ArticularContactForce>(frc_path);

        std::string casting_mesh_name = frc.getConnectee<Smith2018ContactMesh>("casting_mesh").getName();
        std::string target_mesh_name = frc.getConnectee<Smith2018ContactMesh>("target_mesh").getName();

        if (mesh_name == casting_mesh_name) {            
            mesh_type = "casting";
            mesh = frc.getConnectee<Smith2018ContactMesh>("casting_mesh");
        }
        else if (mesh_name == target_mesh_name) {            
            mesh_type = "target";
            mesh = frc.getConnectee<Smith2018ContactMesh>("target_mesh");
        }
        else {
            continue;
        }

        int nVectorDouble = -1;
        for (std::string output_name : _contact_output_vector_double_names) {
            nVectorDouble++;
            std::vector<std::string> output_name_split = split_string(output_name, "_");
            std::string output_mesh_type = output_name_split[0];
            std::string output_data_type = output_name_split[1];
            std::string output_data_name = "";
                
            for (int i = 2; i < output_name_split.size(); ++i) {
                if (i == 2) {
                    output_data_name = output_name_split[i];
                }
                else {
                    output_data_name = output_data_name + "_" + output_name_split[i];
                }
            }

            if (output_mesh_type != mesh_type) {
                continue;
            }
            
            if (output_data_type == "tri") {

                if (getProperty_tri_output_data_names().findIndex(output_data_name) == -1
                    && get_tri_output_data_names(0) != "all") {
                    continue;
                }

                //Seperate data for each contact force
                triDataNames.push_back(
                    "tri." + output_data_name + "_" + frc.getName());
                triData.push_back(
                    _contact_output_vector_double_values[nFrc][nVectorDouble]);

                //Combined data for all contacts visualized on one mesh
                int data_index;
				if (contains_string(triDataNames, "tri." + output_data_name,data_index)) {
					triData[data_index] += _contact_output_vector_double_values[nFrc][nVectorDouble];
				}
				else {
					triDataNames.push_back("tri." + output_data_name);
                    triData.push_back(_contact_output_vector_double_values[nFrc][nVectorDouble]);
                    std::cout <<  _contact_output_vector_double_values[nFrc][nVectorDouble].nrow() << _contact_output_vector_double_values[nFrc][nVectorDouble].ncol() << std::endl;
				}
            }
            
        }
        //Variable Cartilage Properties
	    if ((getProperty_vtp_contact_properties().findIndex("thickness") != -1
            || getProperty_vtp_contact_properties().findIndex("all") != -1)
		    && !contains_string(triDataNames, "tri.thickness")){            
		
		    SimTK::Matrix thickness_matrix(_n_frames, mesh.getNumFaces());
		    for (int i = 0; i < _n_frames; ++i) {                
                thickness_matrix[i] =  ~mesh.getTriangleThickness();
		    }

		    triDataNames.push_back("tri.thickness");            
		    triData.push_back(thickness_matrix);
	    }	    
    }
    if ((getProperty_vtp_contact_properties().findIndex("elastic_modulus") != -1
        || getProperty_vtp_contact_properties().findIndex("all") != -1)
        && !contains_string(triDataNames, "tri.elastic_modulus")) {
            
            SimTK::Matrix E_matrix(_n_frames, mesh.getNumFaces());
		    for (int i = 0; i < _n_frames; ++i) {
			    E_matrix[i] = ~mesh.getTriangleElasticModulus();
		    }
		    triDataNames.push_back("tri.elastic_modulus");
		    triData.push_back(E_matrix);
	}
	if ((getProperty_vtp_contact_properties().findIndex("poissons_ratio") != -1
        || getProperty_vtp_contact_properties().findIndex("all") != -1)
        && !contains_string(triDataNames, "tri.poissons_ratio")) {

		SimTK::Matrix v_matrix(_n_frames, mesh.getNumFaces());
		for (int i = 0; i < _n_frames; ++i) {
			v_matrix[i] = ~mesh.getTrianglePoissonsRatio();
		}
		triDataNames.push_back("tri.poissons_ratio");
		triData.push_back(v_matrix);
	}
    if ((getProperty_vtp_contact_properties().findIndex("area") != -1
        || getProperty_vtp_contact_properties().findIndex("all") != -1)
        && !contains_string(triDataNames, "tri.area")) {

		SimTK::Matrix area_matrix(_n_frames, mesh.getNumFaces());
		for (int i = 0; i < _n_frames; ++i) {
			area_matrix[i] = ~mesh.getTriangleAreas();
		}
		triDataNames.push_back("tri.area");
		triData.push_back(area_matrix);
	}
}

void JointMechanicsTool::writeVTPFile(const std::string& mesh_name,
	const std::vector<std::string>& contact_names, bool isDynamic) {

    std::string file_path = get_results_directory();
    std::string base_name = get_results_file_basename();

    std::string frame = split_string(get_vtp_frame(), "/").back();
    std::string origin = split_string(get_vtp_origin(), "/").back();

	//Collect data
	std::vector<SimTK::Matrix> triData, vertexData;
	std::vector<std::string> triDataNames, vertexDataNames;

	collectMeshContactOutputData(mesh_name,
        triData, triDataNames, vertexData, vertexDataNames);

	//Mesh face connectivity
	SimTK::PolygonalMesh mesh = _model->getComponent<Smith2018ContactMesh>
		(mesh_name).getPolygonalMesh();

	SimTK::Matrix mesh_faces(mesh.getNumFaces(), mesh.getNumVerticesForFace(0));

	for (int j = 0; j < mesh.getNumFaces(); ++j) {
		for (int k = 0; k < mesh.getNumVerticesForFace(0); ++k) {
			mesh_faces(j, k) = mesh.getFaceVertex(j, k);
		}
	}

	for (int frame_num = 0; frame_num < _n_frames; ++frame_num) {
		//Write file
		VTPFileAdapter* mesh_vtp = new VTPFileAdapter();
		mesh_vtp->setDataFormat("binary");	
        //mesh_vtp->setDataFormat("ascii");
		for (int i = 0; i < triDataNames.size(); ++i) {
			mesh_vtp->appendFaceData(triDataNames[i], ~triData[i][frame_num]);
		}


		if (isDynamic) {
			int mesh_index;
			contains_string(_contact_mesh_names, mesh_name, mesh_index);

			mesh_vtp->setPointLocations(_mesh_vertex_locations[mesh_index][frame_num]);
			mesh_vtp->setPolygonConnectivity(mesh_faces);

			mesh_vtp->write(base_name + "_contact_" + mesh_name + "_dynamic_" + frame + "_" + origin,
				file_path + "/", frame_num);
		}
		else { //static
			SimTK::PolygonalMesh poly_mesh =
				_model->getComponent<Smith2018ContactMesh>(mesh_name).getPolygonalMesh();

			mesh_vtp->setPolygonsFromMesh(poly_mesh);

			mesh_vtp->write(base_name + "_contact_" + mesh_name +
				"_static_" + frame, file_path + "/", frame_num);
		}
		delete mesh_vtp;
	}
}

void JointMechanicsTool::writeAttachedGeometryVTPFiles(bool isDynamic) {
    std::string file_path = get_results_directory();
    std::string base_name = get_results_file_basename();

    std::string frame = split_string(get_vtp_frame(), "/").back();
    std::string origin = split_string(get_vtp_origin(), "/").back();

	for (int i = 0; i < _attach_geo_names.size(); ++i) {
		//Face Connectivity
		const SimTK::PolygonalMesh& mesh = _attach_geo_meshes[i];

		SimTK::Matrix mesh_faces(mesh.getNumFaces(), mesh.getNumVerticesForFace(0));

		for (int j = 0; j < mesh.getNumFaces(); ++j) {
			for (int k = 0; k < mesh.getNumVerticesForFace(0); ++k) {
				mesh_faces(j, k) = mesh.getFaceVertex(j, k);
			}
		}

		for (int frame_num = 0; frame_num < _n_frames; ++frame_num) {

			//Write file
			VTPFileAdapter* mesh_vtp = new VTPFileAdapter();
			mesh_vtp->setDataFormat("binary");
            
            if (isDynamic) {
				mesh_vtp->setPointLocations(_attach_geo_vertex_locations[i][frame_num]);
				mesh_vtp->setPolygonConnectivity(mesh_faces);

				mesh_vtp->write(base_name + "_mesh_" + _attach_geo_names[i] + "_dynamic_" +
					frame + "_" + origin, file_path + "/", frame_num);
			}
			else { //static
				mesh_vtp->setPolygonsFromMesh(mesh);
				mesh_vtp->write(base_name + "_mesh_" + _attach_geo_names[i] + "_static_" +
					frame + "_" + origin, file_path + "/", frame_num);
			}
			delete mesh_vtp;
		}
	}
}

void JointMechanicsTool::writeLineVTPFiles(std::string line_name,
    const SimTK::Vector& nPoints, const SimTK::Matrix_<SimTK::Vec3>& path_points,
    const std::vector<std::string>& output_double_names, const SimTK::Matrix& output_double_values) 
{
    for (int i = 0; i < _n_frames; ++i) {
	    int nPathPoints = nPoints.get(i);
			
	    VTPFileAdapter* mesh_vtp = new VTPFileAdapter();
	    mesh_vtp->setDataFormat("binary");
       

	    //Collect points
	    SimTK::RowVector_<SimTK::Vec3> points(nPathPoints);
	    SimTK::Vector lines(nPathPoints);

	    for (int k = 0; k < nPathPoints; k++) {
		    points(k) = path_points.get(i, k);
		    lines(k) = k;
	    }
				
	    mesh_vtp->setPointLocations(points);
	    mesh_vtp->setLineConnectivity(lines);

	    //Collect Data
        for (int k = 0; k < output_double_names.size(); ++k) {
            SimTK::Vector point_data(nPathPoints, output_double_values[i][k]);
		    mesh_vtp->appendPointData(output_double_names[k], point_data);
	    }

        //Write File
        std::string frame = split_string(get_vtp_frame(), "/").back();
        std::string origin = split_string(get_vtp_origin(), "/").back();

	    mesh_vtp->write(get_results_file_basename() + "_" + line_name + "_" + 
            frame + "_" + origin, get_results_directory() + "/", i);	
	    delete mesh_vtp;
    }
}

void JointMechanicsTool::writeH5File(
	const std::string &aBaseName, const std::string &aDir)
{
	H5FileAdapter h5_adapter;

	const std::string h5_file{ aDir + "/" + aBaseName + ".h5" };
	h5_adapter.open(h5_file);
    h5_adapter.writeTimeDataSet(_time);

	//Write States Data
	if (get_h5_states_data()) {
		StatesReporter& states_analysis = dynamic_cast<StatesReporter&>(_model->updAnalysisSet().get("states_analysis"));
		const TimeSeriesTable& states_table = states_analysis.getStatesStorage().exportToTable();
		h5_adapter.writeStatesDataSet(states_table);
	}

    //Write coordinate data
	if (get_h5_kinematics_data()) {
        h5_adapter.writeComponentGroupDataSet("Coordinates", _coordinate_names, _coordinate_output_double_names, _coordinate_output_double_values);
	}

	//Write Muscle Data
	if (get_h5_muscle_data()) {
		h5_adapter.writeComponentGroupDataSet("Muscles",_muscle_names, _muscle_output_double_names, _muscle_output_double_values);		
	}

	//Write Ligament Data
	if (get_h5_ligament_data()) {
        h5_adapter.writeComponentGroupDataSet("Ligaments",_ligament_names, _ligament_output_double_names, _ligament_output_double_values);
	}

	//Write Contact Data
    /*if (get_h5_raw_contact_data() || get_h5_summary_contact_data()) {
        std::string cnt_group_name{ "/Contacts" };
	    h5_adapter.createGroup(cnt_group_name);

        //Contact Force
        for (int i = 0; i < _contact_force_paths.size(); ++i) {
            std::string contact_path = _contact_force_paths[i];

            Smith2018ArticularContactForce& frc = _model->
                updComponent<Smith2018ArticularContactForce>(contact_path);

            std::string contact_name = frc.getName();

            h5_adapter.createGroup(cnt_group_name + "/" + contact_name);

            //Contact Meshes
            std::vector<std::string> mesh_names;
            mesh_names.push_back(frc.getConnectee<Smith2018ContactMesh>("target_mesh").getName());
            mesh_names.push_back(frc.getConnectee<Smith2018ContactMesh>("casting_mesh").getName());

            for (std::string mesh_name : mesh_names) {
                std::string mesh_path = cnt_group_name + "/" + contact_name + "/" + mesh_name;
                h5_adapter.createGroup(mesh_path);
                            
                //Write Summary Contact Metrics 
                if (get_h5_summary_contact_data()) {
                    std::vector<std::string> data_regions;
                    data_regions.push_back("total");
                    if (get_h5_medial_lateral_summary()) {
                        data_regions.push_back("mediallateral");
                    }

                    for (std::string region : data_regions) {
                        h5_adapter.createGroup(mesh_path + "/" + region);

                        //Write output doubles
                        h5_adapter.writeDataSetSimTKMatrixColumns(matrix_data, report_labels);

                        //Write output Vec3
                        h5_adapter.writeDataSetSimTKMatrixVec3Columns(report_vec3.getTable().getMatrix().getAsMatrix(), report_vec3_labels);
                    }
                }
                //Write pressure/proximity on every triangle
                if (get_h5_raw_contact_data()) {
                    h5_adapter.createGroup(mesh_path + "/tri");

                    if (get_output_pressure()) {
                        std::string data_path = mesh_path + "/tri/" + "pressure";
                        h5_adapter.writeDataSetVector(report.getTable(), data_path);
                    }
                    if (get_output_proximity()) {                        
                        std::string data_path = mesh_path + "/tri/" + "proximity";
                        h5_adapter.writeDataSetVector(report.getTable(), data_path);
                    }
                    
                }
            }
        }
    }
*/
	h5_adapter.close();

}

void JointMechanicsTool::loadModel(const std::string &aToolSetupFileName)
{
    
	OPENSIM_THROW_IF(get_model_file().empty(), Exception,
            "No model file was specified (<model_file> element is empty) in "
            "the Setup file. ");
    std::string saveWorkingDirectory = IO::getCwd();
    std::string directoryOfSetupFile = IO::getParentDirectory(aToolSetupFileName);
    IO::chDir(directoryOfSetupFile);

    std::cout<<"JointMechanicsTool "<< getName() <<" loading model '"<<get_model_file() <<"'"<< std::endl;

	Model *model = 0;

    try {
        model = new Model(get_model_file());
		model->finalizeFromProperties();
        
    } catch(...) { // Properly restore current directory if an exception is thrown
        IO::chDir(saveWorkingDirectory);
        throw;
    }
    _model = model;
    IO::chDir(saveWorkingDirectory);
}
