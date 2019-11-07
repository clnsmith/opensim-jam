#ifndef OPENSIM_SMITH2018_CONTACT_MESH_H_
#define OPENSIM_SMITH2018_CONTACT_MESH_H_
/* -------------------------------------------------------------------------- *
 *                           Smith2018ContactMesh.h                           *
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


// INCLUDE
#include "OpenSim/Common/Object.h"
#include "OpenSim/Simulation/SimbodyEngine/Body.h"
#include "OpenSim/Simulation/Model/ModelComponent.h"
#include "SimTKsimbody.h"
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
#include "osimPluginDLL.h"
#include "simmath/internal/ContactGeometry.h"
/**
* The Simbody 
* OrientedBoundingBox class is used to perform ray intersection tests and the 
* bounding box tree hierarchy construction is adapted from the Simbody Class:
* SimTK::ContactGeometry::TriangularMesh::OBBTreeNodeImpl. A 

*/

namespace OpenSim {
    


	class OSIMPLUGIN_API Smith2018ContactMesh : public ModelComponent {
    //class Smith2018ContactMesh : public ModelComponent {

		OpenSim_DECLARE_CONCRETE_OBJECT(Smith2018ContactMesh, ModelComponent)

	public:
        class OBBTreeNode;
		//=============================================================================
		// PROPERTIES
		//=============================================================================
		OpenSim_DECLARE_PROPERTY(file_name, std::string,
			"Path to mesh geometry file (supports .obj, .stl, .vtp). ")
		OpenSim_DECLARE_PROPERTY(mesh_frame, PhysicalOffsetFrame,
			"Local mesh frame to locate mesh in parent body.")
		OpenSim_DECLARE_PROPERTY(display_preference, int,
			"0:Hide 1:Wire 3:Flat 4:Shaded")
		OpenSim_DECLARE_PROPERTY(scale_factors,SimTK::Vec3,
			"[x,y,z] scale factors applied to mesh vertex locations.")
        OpenSim_DECLARE_PROPERTY(min_proximity, double,
			"Minimum overlap depth of meshes to define triangles in contact")
        OpenSim_DECLARE_PROPERTY(max_proximity, double,
			"Maximum overlap depth of meshes to define triangles in contact")           
        OpenSim_DECLARE_OPTIONAL_PROPERTY(mesh_back_file, std::string,
			"Path to backside (bone) mesh geometry file (supports .obj, .stl, .vtp). ")
		OpenSim_DECLARE_OPTIONAL_PROPERTY(material_properties_file, std::string,
			"Path to backside (bone) mesh geometry file (supports .obj, .stl, .vtp). ")
		OpenSim_DECLARE_OPTIONAL_PROPERTY(min_thickness, double,
			"Minimum thickness of cartilage [m] for variable cartilage thickness")
		OpenSim_DECLARE_OPTIONAL_PROPERTY(max_thickness, double,
			"Minimum thickness of cartilage [m] for variable cartilage thickness")
		OpenSim_DECLARE_LIST_PROPERTY_SIZE(color, double, 3,
			"Display Color to apply to the contact geometry.")
		OpenSim_DECLARE_SOCKET(parent_frame, PhysicalFrame,
			"The frame to which this geometry is attached.")
		OpenSim_DECLARE_SOCKET(scale_frame, PhysicalFrame,
			"The frame whose scale factors are used to scale the mesh.")

		//=============================================================================
		// METHODS
		//=============================================================================
	public:
		// CONSTRUCTION
		/** Construct an empty ContactGeometry. */
		Smith2018ContactMesh();

		/** This constructor connects this ContactGeometry to the provided `frame`,
		* and uses the default location and orientation (both `Vec3(0)`).
		*
		* @param frame        the PhysicalFrame this geometry is attached to;
		*/
		Smith2018ContactMesh(const std::string& name, const std::string& mesh_file, const PhysicalFrame& frame);

		Smith2018ContactMesh(const std::string& name, const std::string& mesh_file, const PhysicalFrame& frame,
			const SimTK::Vec3& location, const SimTK::Vec3& orientation);

		Smith2018ContactMesh(const std::string& name, const std::string& mesh_file, const PhysicalFrame& frame,
			const SimTK::Vec3& location, const SimTK::Vec3& orientation,
			const std::string& mesh_back_file, double  min_thickness, double max_thickness);
               
		const int getDisplayPreference();
		void setDisplayPreference(const int dispPref);

		const SimTK::PolygonalMesh& getPolygonalMesh() const {
			return _mesh;}

		int getNumFaces() const {
			return _mesh.getNumFaces();}

		int getNumVertices() const {		
            return _mesh.getNumVertices();}

		SimTK::Vector getNeighborTris(int tri, int& nNeighborTri) const;

		const std::vector<std::vector<int>>& getRegionalTriangleIndices() const {
			return _regional_tri_ind;
		}

		const SimTK::Vector& getTriangleThickness() const {
			return _tri_thickness;
		}
        void setUniformTriangleThickness(double thickness) {
            _tri_thickness = thickness;
		}

		const SimTK::Vector& getTriangleElasticModulus() const {
			return _tri_elastic_modulus;
		}

		const SimTK::Vector& getTrianglePoissonsRatio() const {
			return _tri_poissons_ratio;
		}
        const SimTK::Vector& Smith2018ContactMesh::getTriangleAreas() const {
	        return _tri_area;}

        const SimTK::Vector_<SimTK::Vec3>& Smith2018ContactMesh::getTriangleCenters() const {
	        return _tri_center;}

        const SimTK::Vector_<SimTK::UnitVec3>& Smith2018ContactMesh::getTriangleNormals() const {
	        return _tri_normal;}

        const SimTK::Matrix_<SimTK::Vec3>& Smith2018ContactMesh::getFaceVertexLocations() const {
	        return _face_vertex_locations;}

        const SimTK::Vector_<SimTK::Vec3>& Smith2018ContactMesh::getVertexLocations() const {
	        return _vertex_locations;}

        const OBBTreeNode& getObbTreeNode() const {
            return _obb;
        }
        int getObbNumTriangles() const {
            return _obb._numTriangles;
        }
		virtual void scale(const ScaleSet& aScaleSet);

        bool rayIntersectMesh(
            const SimTK::Vec3& origin, const SimTK::UnitVec3& direction, 
            int& tri, SimTK::Vec3 intersection_point,
            SimTK::Real& distance) const;

	protected:

	private:
		// INITIALIZATION
		void setNull();
		void constructProperties();
		void extendFinalizeFromProperties() override;		
        void extendAddToSystem(SimTK::MultibodySystem &system) const override;
        void extendInitStateFromProperties(SimTK::State &state) const override;
		void extendRealizeReport(const SimTK::State &state) const override;
		void extendScale(const SimTK::State& s, const ScaleSet& scaleSet) override;
		void initializeMesh();
		std::string findMeshFile(const std::string& file);
		
        void createObbTree
            (OBBTreeNode& node, const SimTK::PolygonalMesh& mesh,
            const SimTK::Array_<int>& faceIndices);

        void splitObbAxis(const SimTK::PolygonalMesh& mesh,
            const SimTK::Array_<int>& parentIndices,
            SimTK::Array_<int>& child1Indices,
            SimTK::Array_<int>& child2Indices, int axis);

		void computeVariableCartilageThickness();

		// Member Variables
		SimTK::PolygonalMesh _mesh;
		SimTK::PolygonalMesh _mesh_back;
		SimTK::Vector_<SimTK::Vec3> _tri_center;
		SimTK::Vector_<SimTK::UnitVec3> _tri_normal;
		SimTK::Vector _tri_area;
        std::vector<std::vector<int>> _regional_tri_ind;
        std::vector<int> _regional_n_tri;		
		SimTK::Matrix _tri_neighbors;
		SimTK::Vector _n_tri_neighbors;
		SimTK::Vector_<SimTK::Vec3> _vertex_locations;
		SimTK::Matrix_<SimTK::Vec3> _face_vertex_locations;        
		SimTK::Vector _tri_thickness;
		SimTK::Vector _tri_elastic_modulus;
		SimTK::Vector _tri_poissons_ratio;
		bool mesh_is_cached;

    //=========================================================================
    //                            OBB TREE NODE
    //=========================================================================
        
    public:
        class OBBTreeNode {
            public:
                OBBTreeNode() : _child1(NULL), _child2(NULL), _numTriangles(0) {
                }
                OBBTreeNode(const OBBTreeNode& copy);
                ~OBBTreeNode();
                
                bool rayIntersectOBB(
                    const SimTK::PolygonalMesh& mesh,
                    const SimTK::Vec3& origin,
                    const SimTK::UnitVec3& direction, 
                    int& tri_index, SimTK::Vec3& intersection_point,  
                    double& distance) const;

                bool rayIntersectTri(
                    const SimTK::PolygonalMesh& mesh,
                    SimTK::Vec3 origin, SimTK::Vec3 direction,
                    int tri_index,
                    SimTK::Vec3& intersection_pt, double& distance) const;
                
                const SimTK::OrientedBoundingBox& getBounds() const ;
                bool isLeafNode() const;
                const OBBTreeNode getFirstChildNode() const ;
                const OBBTreeNode getSecondChildNode() const;
                const SimTK::Array_<int>& getTriangles() const;                       
                int getNumTriangles() const;

                SimTK::OrientedBoundingBox _bounds;
                OBBTreeNode* _child1;
                OBBTreeNode* _child2;
                SimTK::Array_<int> _triangles;
                int _numTriangles;
                    
        };
        //friend class Smith2018ContactMesh;
        
        OBBTreeNode _obb;
        OBBTreeNode _back_obb;

		//=============================================================================
	};  // END of class ContactGeometry
		//=============================================================================
		//=============================================================================



} // end of namespace OpenSim

#endif // OPENSIM_SMITH2018_CONTACT_MESH_H_
