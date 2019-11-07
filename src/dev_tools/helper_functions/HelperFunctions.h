#ifndef OPENSIM_HELPER_FUNCTIONS_H_
#define OPENSIM_HELPER_FUNCTIONS_H_
/* -------------------------------------------------------------------------- *
 *                             HelperFunctions.h                              *
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

//=============================================================================
//=============================================================================
//namespace OpenSim {
/*
#ifdef _WIN32
#  ifdef WISCO_API_EXPORTS
#    define WISCO_API __declspec(dllexport)
#  else
#    define WISCO_API __declspec(dllimport)
#  endif
#else
#  define WISCO_API
#endif
*/
//=============================================================================
//STRING TOOLS
//=============================================================================
	
/** Split string at delimiter
*/
std::vector<std::string> split_string(std::string s, std::string delimiter);

/*WISCO_API bool contains_string(std::vector<std::string> s_vector, std::string s);
WISCO_API bool contains_string(std::vector<std::string> s_vector, std::string s, int& index);*/
bool contains_string(std::vector<std::string> s_vector, std::string s);
bool contains_string(std::vector<std::string> s_vector, std::string s, int& index);
int find_nearest(std::vector<double> in_vec, double value);
std::string erase_sub_string(std::string mainStr, const std::string & toErase);
//SimTK::Matrix sort_matrix_by_column(SimTK::Matrix& matrix, int col);

//}; //namespace
#endif // #ifndef OPENSIM_HELPER_FUNCTIONS_H_
