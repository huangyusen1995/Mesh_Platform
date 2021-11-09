#pragma once
#pragma once

#include <easy3d/core/surface_mesh.h>
#include <map>

namespace easy3d {
	class  FeatherGraph_CurveGenerate {
	public:
		//! Construct with mesh to be simplified.
		FeatherGraph_CurveGenerate(SurfaceMesh *mesh);

		// destructor
		~FeatherGraph_CurveGenerate();
		void vertex_select();

		void generate_curve();


	public:
		SurfaceMesh *mesh_;
		SurfaceMesh::EdgeProperty<bool> e_feature;
		SurfaceMesh::VertexProperty<bool> v_feature;
		//std::map<SurfaceMesh::Halfedge, SurfaceMesh::Vertex> map_E3;//£¨³ö±ß£¬µã£©
		//std::map<SurfaceMesh::Halfedge, SurfaceMesh::Vertex> map_E2;
		SurfaceMesh::VertexProperty<int> edge_cout;
		//std::map<SurfaceMesh::Vertex, std::vector<SurfaceMesh::Halfedge>> map_E0;
		//std::map<SurfaceMesh::Vertex, int> cout_E0;
		SurfaceMesh::VertexProperty<std::vector<SurfaceMesh::Halfedge>> vertex_edges;
	};

}
