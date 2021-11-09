#include<string>
#include <iostream>
#include"Feature_Graph.h"
#include"MusfordShahAT.h"
#include <easy3d/fileio/surface_mesh_io.h>

using namespace easy3d;
using namespace std;
namespace easy3d
{
	FeatherGraph_CurveGenerate::FeatherGraph_CurveGenerate(SurfaceMesh *mesh) : mesh_(mesh)
	{
		e_feature = mesh_->get_edge_property<bool>("e:feature");
		if (e_feature == NULL)
		{
			cout << "have no e_feature" << endl;
			return;
		}
		v_feature = mesh_->get_vertex_property<bool>("v:feature");
		if (v_feature == NULL)
		{
			cout << "have no v_feature" << endl;
			return;
		}
	}

	FeatherGraph_CurveGenerate::~FeatherGraph_CurveGenerate() {}

	void FeatherGraph_CurveGenerate::vertex_select()   //��ÿ�������ڱߣ���������e_feature=1�ıߡ�
	{
		int cout = 0;
		SurfaceMesh::Edge ee;
		SurfaceMesh::Vertex v0;
		SurfaceMesh::Vertex v1;
		SurfaceMesh::Halfedge h0;
		SurfaceMesh::Halfedge h1;
		edge_cout = mesh_->vertex_property<int>("v:edge_cout", 0);
		vertex_edges = mesh_->vertex_property<std::vector<SurfaceMesh::Halfedge>>("v:vertex_edges");

		for (auto e : mesh_->edges())
		{
			if (e_feature[e] == 1)
			{
				h0 = mesh_->halfedge(e, 0); //e�İ��
				v0 = mesh_->source(h0);     //��ߵ����
				h1 = mesh_->halfedge(e, 1); //e����һ�����
				v1 = mesh_->source(h1);     //��ߵ����
				//map_E3.insert(std::pair < SurfaceMesh::Halfedge, SurfaceMesh::Vertex>(h0, v0));
				//map_E2.insert(std::pair < SurfaceMesh::Halfedge, SurfaceMesh::Vertex>(h1, v1));
				edge_cout[v0] ++;
				edge_cout[v1] ++;
				vertex_edges[v0].push_back(h0);
				vertex_edges[v1].push_back(h1);
				//map_E0[v0].push_back(h0);
				//cout_E0[v0] ++;
			}
		}

		/*for (auto v : mesh_->vertices())
		{
			cout = 0;
			for (auto vv : mesh_->vertices(v))
			{
				ee = mesh_->find_edge(v, vv);
				if (e_feature[ee] == 1) cout++;
			}
			if (cout >= 3){}
			if (cout == 2){}
			*/
	}


	void FeatherGraph_CurveGenerate::generate_curve()
	{
		std::vector<SurfaceMesh::Vertex> V3;
		std::vector<SurfaceMesh::Vertex> V2;
		for (auto v : mesh_->vertices())
		{

			if (edge_cout[v] >= 3)  V3.push_back(v); //std::cout << v << "\n";
			else if (edge_cout[v] == 2) V2.push_back(v);
			else continue;
		}
		//**************************��ʼ��������V3*************************************

		std::vector<std::vector<SurfaceMesh::Vertex>> curves;
		std::vector<SurfaceMesh::Vertex> now_curve;
		SurfaceMesh::Vertex now_vertex;
		SurfaceMesh::Halfedge now_halfedge;
		int cur_i = 0;
		for (int i = 0; i < V3.size(); i++)
		{

			while (int(vertex_edges[V3[i]].size()))  //V3[i]���ڰ�� = vertex_edges[V3[i]][j]
			{

				//curves[cur_i].push_back(V3[i]);//�������߶˵�V3[i]
				now_curve.push_back(V3[i]);
				now_halfedge = vertex_edges[V3[i]][0];

				//vertex_edges[V3[i]].pop_back();
				//std::cout << now_halfedge.idx() << "\n";
				;				now_vertex = mesh_->target(now_halfedge);//��һ�γ��ߵĶ�Ӧ��

				while (std::find(V3.begin(), V3.end(), now_vertex) == V3.end()) //now_vertex������V3��
				{
					//curves[i + j].push_back(now_vertex);//���������м��
					if (std::find(V2.begin(), V2.end(), now_vertex) != V2.end()) //now_vertex����V2��
					{
						if (mesh_->opposite(vertex_edges[now_vertex][0]) != now_halfedge)
						{
							now_curve.push_back(now_vertex);//���������м��
							edge_cout[now_vertex] = 0;//ɾ��V2����;��¼�µĵ�

							now_halfedge = vertex_edges[now_vertex][0];
							now_vertex = mesh_->target(now_halfedge);
						}
						else
						{
							now_curve.push_back(now_vertex);//���������м��
							edge_cout[now_vertex] = 0;//ɾ��V2����;��¼�µĵ�

							now_halfedge = vertex_edges[now_vertex][1];
							now_vertex = mesh_->target(now_halfedge);
						}
					}
					else                 //now_vertex������V2�У�����V1
					{
						//curves[i + j].pop_back();//�����һ��now_vertex����ɾ��
						break;
					}
				}
				now_curve.push_back(now_vertex);//�������߶˵�now_vertex
				//now_vertex:mesh_->opposite(now_halfedge)
				//V3[i]:vertex_edges[V3[i]][j]
				//********************��ʼɾ����һ�����߶˵�ĳ����********************
				for (vector<SurfaceMesh::Halfedge>::iterator it = vertex_edges[now_vertex].begin(); it != vertex_edges[now_vertex].end(); )
				{
					if (*it == mesh_->opposite(now_halfedge)) it = vertex_edges[now_vertex].erase(it);
					else ++it;
				}
				//std::cout << "vertex_edges size befor:" << vertex_edges[V3[i]].size() << "\n";

				vertex_edges[V3[i]].erase(vertex_edges[V3[i]].begin());
				/*for (vector<SurfaceMesh::Halfedge>::iterator it = vertex_edges[V3[i]].begin(); it != vertex_edges[V3[i]].end(); )
				{
					if (*it == vertex_edges[V3[i]][0]) it = vertex_edges[V3[i]].erase(it);
					else ++it;
				}*/
				//std::cout << "vertex_edges size after:" << vertex_edges[V3[i]].size() << "\n";
				curves.push_back(now_curve);
				now_curve.clear();
				cur_i++;

				



			}
		}
		std::cout << "v3_curves_nember:" << cur_i << "\n";
		//***************************��ʼ����V2******************************************
		now_curve.clear();
		V2.clear();
		for (auto v : mesh_->vertices())
		{
			if (edge_cout[v] == 2) V2.push_back(v);
		}
		SurfaceMesh::Vertex start_vertex;
		while (int(V2.size()))
		{
			start_vertex = V2[0];
			now_curve.push_back(start_vertex);//�������߶˵�V2[i]
			edge_cout[start_vertex] = 0;
			for (int j = 0; j <2; j++)  //V3[i]���ڰ�� = vertex_edges[V2[i]][j]
			{
				now_halfedge = vertex_edges[start_vertex][j];
				
				now_vertex = mesh_->target(now_halfedge);//��һ�γ��ߵĶ�Ӧ��
				
				while (std::find(V2.begin(), V2.end(), now_vertex) != V2.end()) //now_vertex����V2��
				{
					if (start_vertex == now_vertex) //v2�ջ�
					{ 
						j = 2; 
						break; 
					}  
					if (mesh_->opposite(vertex_edges[now_vertex][0]) != now_halfedge)
					{
						now_curve.push_back(now_vertex);//���������м��
						edge_cout[now_vertex] = 0;//ɾ��V2����;��¼�µĵ�

						now_halfedge = vertex_edges[now_vertex][0];
						now_vertex = mesh_->target(now_halfedge);
					}
					else
					{
						now_curve.push_back(now_vertex);//���������м��
						edge_cout[now_vertex] = 0;//ɾ��V2����;��¼�µĵ�

						now_halfedge = vertex_edges[now_vertex][1];
						now_vertex = mesh_->target(now_halfedge);
					}
				}
				now_curve.push_back(now_vertex);//��������V1�˵�now_vertex
			}
			//***********************�ڵ�ǰ�����ڵĵ��V2��ɾ��**************************
	       /*std::vector<SurfaceMesh::Vertex>::iterator it1;
			for (int k = 0; k < now_curve.size(); k++)
			{
				for (it1 = V2.begin(); it1 != V2.end();)
				{
					if (*it1 == curves[cur_i][k])
					{
						it1 = V2.erase(it1);
					}
					else ++it1;
				}
			}*/
			V2.clear();
			for (auto v : mesh_->vertices())
			{
				if (edge_cout[v] == 2) V2.push_back(v);
			}
			curves.push_back(now_curve);
			now_curve.clear();
			cur_i++;
		}
		std::cout << "v2_curves_nember:" << cur_i << "\n";
		//*****************************v_feature��ֵ******************************************
		for (auto v : mesh_->vertices())
		{
			v_feature[v] = false;
		}
		for (int i = 0; i < curves.size(); i++)
		{
			//std::cout << curves[i][0] << "||"<<curves[i][int(curves[i].size()) - 1]<<"\n";
			v_feature[curves[i][0]] = true;
			v_feature[curves[i][int(curves[i].size())-1]] = true;
		}
	}
}
