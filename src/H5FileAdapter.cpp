/* -------------------------------------------------------------------------- *
 *                             H5FileAdapter.cpp                              *
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

#include "H5FileAdapter.h"
#include "HelperFunctions.h"
#include <fstream>

using namespace OpenSim;

H5FileAdapter::H5FileAdapter()
{
	_time_is_empty = true;
}

H5FileAdapter* H5FileAdapter::clone() const
{
	return new H5FileAdapter{ *this };
}

void H5FileAdapter::open(const std::string& file_name) 
{
	_file = H5::H5File(file_name, H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
}

void H5FileAdapter::close() {
	_file.close();
}



void H5FileAdapter::createGroup(const std::string& group_name) {
	_file.createGroup(group_name);
}
	
void H5FileAdapter::writeDataSet(const TimeSeriesTable& table, const std::string dataset_path) 
{
		
	SimTK::Matrix data_matrix = table.getMatrix();

	hsize_t dim_data[1];
	dim_data[0] = data_matrix.nrow();
		
	for (int i = 0; i < data_matrix.ncol(); ++i) {
		H5::DataSpace dataspace(1, dim_data, dim_data);
		H5::PredType datatype(H5::PredType::NATIVE_DOUBLE);
		H5::DataSet dataset = _file.createDataSet(dataset_path, datatype, dataspace);

		//Allocate space for data
		double* data = (double*)malloc(dim_data[0] * sizeof(double));

		//Set Data Array
		for (int r = 0; r < dim_data[0]; ++r) {
				data[r] = data_matrix(r,i);
		}
		
		dataset.write(&data[0], datatype);

		//Free dynamically allocated memory
		free(data);
	}
}

void H5FileAdapter::writeDataSetVec3(const TimeSeriesTableVec3& table, const std::string dataset_path)
{
		
	SimTK::Matrix_<SimTK::Vec3> data_matrix = table.getMatrix();

	hsize_t dim_data[2];
	dim_data[0] = data_matrix.nrow();
	dim_data[1] = 3;

	for (int i = 0; i < data_matrix.ncol(); ++i) {

		H5::DataSpace dataspace(2, dim_data, dim_data);
		H5::PredType datatype(H5::PredType::NATIVE_DOUBLE);
		H5::DataSet dataset = _file.createDataSet(dataset_path, datatype, dataspace);

		//Allocate space for data
		double** data = (double**)malloc(dim_data[0] * sizeof(double*));
		data[0] = (double*)malloc(dim_data[1] * dim_data[0] * sizeof(double));
		for (int i = 1; i < dim_data[0]; i++) data[i] = data[0] + i*dim_data[1];

		//Set Data Array
		for (int r = 0; r < dim_data[0]; ++r) {
			for (int c = 0; c < 3; ++c) {
				data[r][c] = data_matrix(r, i)(c);
			}
		}

		dataset.write(&data[0][0], datatype);

		//Free dynamically allocated memory
		free(data[0]);
		free(data);
	}
}

void H5FileAdapter::writeDataSetVector(const TimeSeriesTable& table, const std::string dataset_path)
{
	
	SimTK::Matrix data_matrix = table.getMatrix();

	hsize_t dim_data[2];
	dim_data[0] = data_matrix.nrow();
	dim_data[1] = data_matrix.ncol();
		
	H5::DataSpace dataspace(2, dim_data, dim_data);
	H5::PredType datatype(H5::PredType::NATIVE_DOUBLE);
	H5::DataSet dataset = _file.createDataSet(dataset_path, datatype, dataspace);

	//Allocate space for data
	double** data = (double**)malloc(dim_data[0] * sizeof(double*));
	data[0] = (double*)malloc(dim_data[1] * dim_data[0] * sizeof(double));
	for (int i = 1; i < dim_data[0]; i++) data[i] = data[0] + i*dim_data[1];

	//Set Data Array
	for (int r = 0; r < dim_data[0]; ++r) {
		for (int c = 0; c < dim_data[1]; ++c) {
			data[r][c] = data_matrix(r, c);
		}
	}

	dataset.write(&data[0][0], datatype);

	//Free dynamically allocated memory
	free(data[0]);
	free(data);
		
}

void H5FileAdapter::writeStatesDataSet(const TimeSeriesTable& table) {
	std::string states_group_name = "/States";
	_file.createGroup(states_group_name);

	std::vector<std::string> labels = table.getColumnLabels();
	
	SimTK::Matrix data_matrix = table.getMatrix();

	hsize_t dim_data[1];
	dim_data[0] = data_matrix.nrow();

	for (int i = 0; i < data_matrix.ncol(); ++i) {
		SimTK::String label(labels[i]);

		label.replaceAllChar('/','_');
						
		std::string dataset_path = states_group_name + "/" + label;
			
		H5::DataSpace dataspace(1, dim_data, dim_data);
		H5::PredType datatype(H5::PredType::NATIVE_DOUBLE);
		H5::DataSet dataset = _file.createDataSet(dataset_path, datatype, dataspace);

		//Allocate space for data
		double* data = (double*)malloc(dim_data[0] * sizeof(double));

		//Set Data Array
		for (int r = 0; r < dim_data[0]; ++r) {
			data[r] = data_matrix(r, i);
		}

		dataset.write(&data[0], datatype);

		//Free dynamically allocated memory
		free(data);
	}
}

void H5FileAdapter::writeComponentGroupDataSet(std::string group_name,
    std::vector<std::string> names,
	std::vector<std::string> output_double_names,
    std::vector<SimTK::Matrix> output_double_values)
{
	_file.createGroup(group_name);

    int i = 0;
    for (std::string lig_name : names) {
        std::string lig_group = group_name + "/" + lig_name;
		_file.createGroup(lig_group);

        int j = 0;
        for (std::string data_label : output_double_names) {

            SimTK::Vector data = output_double_values[i](j);
			writeDataSetSimTKVector(data, lig_group + "/" + data_label);
            j++;
        }
        i++;
    }
}
    
void H5FileAdapter::writeDataSetSimTKVector(const SimTK::Vector& data_vector, const std::string dataset_path) {
	hsize_t dim_data[1];
	dim_data[0] = data_vector.size();

	H5::DataSpace dataspace(1, dim_data, dim_data);
	H5::PredType datatype(H5::PredType::NATIVE_DOUBLE);
	H5::DataSet dataset = _file.createDataSet(dataset_path, datatype, dataspace);

	//Allocate space for data
	double* data = (double*)malloc(dim_data[0] * sizeof(double));

	//Set Data Array
	for (int r = 0; r < dim_data[0]; ++r) {
		data[r] = data_vector(r);
	}

	dataset.write(&data[0], datatype);

	//Free dynamically allocated memory
	free(data);
}

void H5FileAdapter::writeDataSetSimTKVectorVec3(const SimTK::Vector_<SimTK::Vec3>& data_vector, const std::string dataset_path) {
	hsize_t dim_data[2];
	dim_data[0] = data_vector.nrow();
	dim_data[1] = 3;
		
	H5::DataSpace dataspace(2, dim_data, dim_data);
	H5::PredType datatype(H5::PredType::NATIVE_DOUBLE);
	H5::DataSet dataset = _file.createDataSet(dataset_path, datatype, dataspace);

	//Allocate space for data
	double** data = (double**)malloc(dim_data[0] * sizeof(double*));
	data[0] = (double*)malloc(dim_data[1] * dim_data[0] * sizeof(double));
	for (int i = 1; i < dim_data[0]; i++) data[i] = data[0] + i*dim_data[1];

	//Set Data Array
	for (int r = 0; r < dim_data[0]; ++r) {
		for (int c = 0; c < 3; ++c) {
			data[r][c] = data_vector(r)(c);
		}
	}

	dataset.write(&data[0][0], datatype);

	//Free dynamically allocated memory
	free(data[0]);
	free(data);
}

void H5FileAdapter::writeDataSetSimTKMatrixColumns(const SimTK::Matrix& data, std::vector<std::string> column_dataset_paths) {

	for (int i = 0; i < data.ncol(); ++i) {
		std::string path = column_dataset_paths[i];

		SimTK::Vector data_vec(data.nrow());
		for (int j = 0; j < data.nrow(); ++j) {
			data_vec(j) = data(j, i);
		}
		writeDataSetSimTKVector(data_vec, path);
	}	
}

void H5FileAdapter::writeDataSetSimTKMatrixVec3Columns(const SimTK::Matrix_<SimTK::Vec3>& data, std::vector<std::string> column_dataset_paths) {

	for (int i = 0; i < data.ncol(); ++i) {
		std::string path = column_dataset_paths[i];

		SimTK::Vector_<SimTK::Vec3> data_vec(data.nrow());
		for (int j = 0; j < data.nrow(); ++j) {
			data_vec(j) = data(j, i);
		}
		writeDataSetSimTKVectorVec3(data_vec, path);
	}
}

void H5FileAdapter::writeTimeDataSet(const Array<double>& time) {
	if (_time_is_empty) {

        hsize_t dim_time[1];
		dim_time[0] = time.size();

		H5::DataSpace time_dataspace(1, dim_time, dim_time);
		H5::PredType time_datatype(H5::PredType::NATIVE_DOUBLE);
		H5::DataSet dataset = _file.createDataSet("/time", time_datatype, time_dataspace);

		dataset.write(&time[0], time_datatype);

		_time_is_empty = false;
	}
}
	
H5FileAdapter::OutputTables H5FileAdapter::extendRead(const std::string& fileName) const 
{
    OutputTables output_tables{};
    return output_tables;
};

void H5FileAdapter::extendWrite(const InputTables& tables, const std::string& fileName) const
{
};
