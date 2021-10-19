#pragma once
#include <easy3d/core/surface_mesh.h>
#include ".\3rd_party\eigen\Eigen\Dense"
#include <set>


namespace easy3d {
	class MumfordShah_AT
	{
	public:
		MumfordShah_AT(SurfaceMesh *mesh);
		~MumfordShah_AT();

		//基础面板参数
		void setParams(double alpha,
			double beta, double threshold,
			double epsilon,
			int normItrNum, int verItrNum);

		//计算算子
		void calculate_operators_indices();
		void filtering();
		void calFeatureEdge();

	protected:
		//找到点的所有连接边的id；
		std::vector<int> FindVertexConnectEdges(SurfaceMesh::Vertex one_v);
		void solveNSubProblem();
		void solveVSubProblem();
		double stopItr();
		double VstopItr();
		double Energy();
		void MSAT_vertexUpdating();
		void MSAT_normalFiltering();
		

	private:
		//面板参数传入的参数
		double m_alpha;
		double m_beta;
		double m_threshold;
		double m_epsilon;
		int m_normal_itr_num;
		int m_vertex_itr_num;

		//计算算子用到的参数
		int m_triangle_num;
		int m_edge_num;
		std::vector<std::vector<int>>        m_triangle_edge_id;
		std::vector<std::vector<int>>		 m_triangle_edge_sgns;
		std::vector<double> m_edges_legth;
		std::vector<std::vector<double>> m_edge_baryEdges;
		std::vector<std::vector<int>>         m_edge_operator_indices;

		//
		std::vector<vec3> m_gradients;

	private:
		SurfaceMesh* mesh_;
		SurfaceMesh::FaceProperty<vec3>   fnormal_;
		SurfaceMesh::FaceProperty<vec3>   normal_filtered;
		SurfaceMesh::FaceProperty<vec3>   previousN;
		SurfaceMesh::FaceProperty<vec3>   N;
		SurfaceMesh::EdgeProperty<double> V;
		SurfaceMesh::EdgeProperty<double> previousV;

	
	public:
	
		SurfaceMesh::VertexProperty<bool> v_feature;
		SurfaceMesh::EdgeProperty<bool> e_feature;
		std::vector<vec3> m_feature_Vertices;
		std::vector<unsigned int> m_feature_edges_indices;
		



	};
}
