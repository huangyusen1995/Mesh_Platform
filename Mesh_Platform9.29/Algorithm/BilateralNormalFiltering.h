
#ifndef EASY3D_TUTORIAL_MESH_FILTER_BF_H
#define EASY3D_TUTORIAL_MESH_FILTER_BF_H

#include <easy3d/core/surface_mesh.h>
#include <easy3d/algo/surface_mesh_geometry.h>

using namespace easy3d;
using namespace std;

class MeshFilteringBF
{

public:
	MeshFilteringBF(SurfaceMesh *mesh);
	~MeshFilteringBF();


	//vector<vector<int>>  FindAllFaceNeighbourFacesII();
	//vector<int> FindFaceNeighborFacesII(int faceIdx);
	//double getAverageEdgeslength();
	//vector<vector<double>>  CaculateTriEdgesLength();

	//vector<SurfaceMesh::Face> FindFaceNeighborFace();

	void BilateralNormalFilter(float theta);
	void vertexUpdate(double specified_threshold);
	void Runfilter(float theta, int iteration, double specified_threshold);
	//int m_nTriangles = mesh_->faces_size();
	//typedef SurfaceMesh::FaceIterator fit;
	//vector<fit>		m_vFacetIterList;

private:
	SurfaceMesh *mesh_;
	SurfaceMesh::FaceProperty<vec3>   fnormal_;
	SurfaceMesh::FaceProperty<vec3>   normal_filtered;
	//vector<SurfaceMesh::Face> normal_filtered;
	//SurfaceMesh::EdgeProperty<vec3>   m_edges;
	SurfaceMesh::VertexProperty<vec3> vertices_in;
	//vector<SurfaceMesh::Face> filtered_normal;


};



#endif  
