#include "BilateralNormalFiltering.h"

using namespace easy3d;
using namespace std;

MeshFilteringBF::MeshFilteringBF(SurfaceMesh *mesh) : mesh_(mesh)
{
	mesh_->update_face_normals();
	fnormal_ = mesh_->get_face_property<vec3>("f:normal");
	normal_filtered = mesh_->get_face_property<vec3>("f:normal");
}


MeshFilteringBF::~MeshFilteringBF()
{
}


void MeshFilteringBF::BilateralNormalFilter(float theta)
{

	SurfaceMesh::Face ff;

	vector<SurfaceMesh::Face> two_ring;//�뵱ǰ�湲���������������
	vector<SurfaceMesh::Face>::iterator or_it, or_end;
	vec3 filtered_normal;

	SurfaceMesh::FaceProperty<vec3> normalst, normalst1;
	normalst1 = mesh_->get_face_property<vec3>("f:normal");
	normalst = mesh_->get_face_property<vec3>("f:normal");

	/*----------------------����mesh��ƽ���߳�--------------------*/
	double  dLength = 0;
	double  sumLength = 0;
	int edge_num = 0;
	edge_num = mesh_->edges_size();
	SurfaceMesh::Edge e;
	for (auto f : mesh_->faces())
	{
		for (auto h : mesh_->halfedges(f))
		{
			e = mesh_->edge(h);
			dLength = mesh_->edge_length(e);
			sumLength += dLength;
		}
	}

	float averagelength = sumLength / edge_num;

	//cout << "averagelength=" << averagelength << endl;
	/*----------------------����mesh��ƽ���߳�--------------------*/

			
	//*------------------------------------------normal filter-----------------------------------------------*//
	for (auto f : mesh_->faces())   //��������ѭ��
	{
		float area_i = geom::triangle_area(mesh_, f);
		//fnormal_[f] = mesh_->compute_face_normal(f);

		//������f������
		vec3 centroid_fi = geom::centroid(mesh_, f);
	

/*-----------------------Ѱ����Ķ�����------------------------*/
		two_ring.clear();
		for (auto v : mesh_->vertices(f))   //��Ķ���
		{
			for (auto h : mesh_->halfedges(v)) //����ĳ����
			{
				ff = mesh_->face(h); //�ҵ�����߽������

				two_ring.push_back(ff);
			}
		}
		//remove repeat faces
		sort(two_ring.begin(), two_ring.end());
		two_ring.erase(unique(two_ring.begin(), two_ring.end()), two_ring.end());
		/*-------------------------Ѱ����Ķ�����-------------------------------------*/
		filtered_normal = vec3(0, 0, 0);
		//������f��II�����淨�����ļ�Ȩ��
		for (or_it = two_ring.begin(), or_end = two_ring.end(); or_it != or_end; ++or_it)
		{
			if (dot(fnormal_[f], fnormal_[*or_it]) > cos(theta * M_PI / 180))
			{
				vec3 centroid_fj = geom::centroid(mesh_, *or_it);				
				float alpha_ij_molecule = norm(centroid_fj - centroid_fi)*norm(centroid_fj - centroid_fi);			
				float alpha_ij_denominator = 2 * averagelength*averagelength;			
				float alpha_ij = exp(-alpha_ij_molecule / alpha_ij_denominator);				
				float beta_ij_molecule = (1 - dot(fnormal_[f], fnormal_[*or_it]))*(1 - dot(fnormal_[f], fnormal_[*or_it]));
				float beta_ij_denominator = (1 - cos(20 * M_PI / 180)) * (1 - cos(20 * 3.14 / 180));
				float beta_ij = exp(-beta_ij_molecule / beta_ij_denominator);		
				filtered_normal += area_i * alpha_ij * beta_ij*fnormal_[*or_it];
			}
		}
		//��f���º�ķ�����
		normalst1[f] = filtered_normal.normalize();
	}

	normalst = normalst1;
	normal_filtered = normalst;
}

void MeshFilteringBF::vertexUpdate(double specified_threshold)
{
	vector<SurfaceMesh::Vertex>  tvs;
	vec3  new_vertex;
	vec3 vertex_displacement = vec3(0, 0, 0);	
	vertices_in = mesh_->get_vertex_property<vec3>("v:point");
	
	vec3 v0, v1, v2, tc, fnormal, pij;
	
	for (auto v : mesh_->vertices())
	{
		int fnum = 0;
		new_vertex = vec3(0, 0, 0);

		for (auto f : mesh_->faces(v))
		{
			vec3 centroid_v_fj = geom::centroid(mesh_, f);//����������j������
			
			fnormal = normal_filtered[f];
			
			pij = fnormal * dot(fnormal, (centroid_v_fj - vertices_in[v]));
			new_vertex += pij;
			fnum++;
		}
		//���µĶ���
		vertex_displacement = (1.0 / fnum)*new_vertex;
		
		if (norm(vertex_displacement) < specified_threshold)
		{
			vertices_in[v] += vertex_displacement;
			
		}
	}
	
}

void MeshFilteringBF::Runfilter(float theta ,int iteration, double specified_threshold)
{
	if (!mesh_->is_triangle_mesh()) {
		std::cerr << "Not a triangle mesh!" << std::endl;
		return;
	}
	

	for (int n = 0; n < iteration; n++)
	{
		BilateralNormalFilter(theta);
		vertexUpdate(specified_threshold);
	}
	cout << "filter end" << endl;
}
