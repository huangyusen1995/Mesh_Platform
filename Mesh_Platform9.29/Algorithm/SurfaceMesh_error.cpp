#include "SurfaceMesh_error.h"

using namespace easy3d;

SurfaceMesh_error::SurfaceMesh_error(SurfaceMesh *mesh) : mesh_(mesh)
{
	mesh_->update_face_normals();
	fnormal = mesh_->get_face_property<vec3>("f:normal");

}



SurfaceMesh_error::~SurfaceMesh_error()
{
}


//ԭʼģ���ϵĶ��㵽��ǰģ������������ƽ������
void SurfaceMesh_error::mean_error(SurfaceMesh *original_mesh)
{
	//auto min_distance = mesh_->add_vertex_property<float>("v:min_distance");
	float total_error = 0.0;

	for(auto v: original_mesh->vertices())
	{
		float min_dist = 100.0;
		
		for(auto f: mesh_->faces())
		{
			vec3 ap = original_mesh->position(v) - mesh_->position(mesh_->target(mesh_->halfedge(f)));  //original_mesh��v���ȥresult_mesh��һ��
			vec3 nn = fnormal[f];                                                                                          //�淨������������
			float dist = (abs(dot(nn, ap)) / sqrt(nn[0] * nn[0] + nn[1] * nn[1] + nn[2] * nn[2]));                        //�����
			if (dist < min_dist) min_dist = dist;
		}
		//min_distance[v] = min_dist;
		total_error += min_dist;
	}
	total_error /= mesh_->n_vertices();
	std::cout << "mean error = " << total_error << std::endl;
}

void SurfaceMesh_error::RMS_error(SurfaceMesh *original_mesh)
{
	float SquareDistance = 0.0;
	float error_RMS = 0.0;

	for (auto v : original_mesh->vertices())
	{
		float min_dist = 100.0;

		for (auto f : mesh_->faces())
		{
			vec3 ap = original_mesh->position(v) - mesh_->position(mesh_->target(mesh_->halfedge(f)));  //original_mesh��v���ȥresult_mesh��һ��
			vec3 nn = fnormal[f];                                                                                         
			float dist = (abs(dot(nn, ap)) / sqrt(nn[0] * nn[0] + nn[1] * nn[1] + nn[2] * nn[2]));  //�����
			if (dist < min_dist) min_dist = dist;
		}
		SquareDistance += pow(min_dist,2);
	}
	int vertices_num= mesh_->n_vertices();
	if (vertices_num != 0)
	{
		error_RMS = sqrt(SquareDistance / vertices_num);
		std::cout << "RMS error = " << error_RMS << std::endl;
	}
	else
	{
		std::cout << "vertices number:0   It's impossible to calculate RMS error" << std::endl;
	}
}


void SurfaceMesh_error::Hausdorff_error(SurfaceMesh *original_mesh)
{
	//��ԭʼģ�͵����ģ�͵ĵ���Hausdorff����
	float distA = 0.0;
	float Hausdorff_distA = 0.0;
	for (auto v : original_mesh->vertices())
	{
		float min_distA = 100.0;
		for (auto f : mesh_->faces())
		{
			//original_mesh��v���ȥresult_mesh��һ��
			vec3 ap = original_mesh->position(v) - mesh_->position(mesh_->target(mesh_->halfedge(f))); 
			//�������
			 distA =  sqrt(ap[0] * ap[0] + ap[1] * ap[1] + ap[2] * ap[2]);  
			if (distA < min_distA)
			{
				min_distA = distA;
			}
		}
		if (min_distA > Hausdorff_distA)
		{
			Hausdorff_distA = min_distA;
		}
	}

	//�ӽ��ģ�͵�ԭʼģ�͵ĵ���Hausdorff����
	float distB = 0.0;
	float Hausdorff_distB = 0.0;
	for (auto vb : mesh_->vertices())
	{
		float min_distB = 100.0;
		for (auto fb : original_mesh->faces())
		{
			//original_mesh��v���ȥresult_mesh��һ��
			vec3 bp = mesh_->position(vb) - original_mesh->position(original_mesh->target(original_mesh->halfedge(fb)));
			//�������
			distB = sqrt(bp[0] * bp[0] + bp[1] * bp[1] + bp[2] * bp[2]);
			if (distB < min_distB)
			{
				min_distB = distB;
			}
		}
		if (min_distB > Hausdorff_distB)
		{
			Hausdorff_distB = min_distB;
		}
	}
	
	float symmetric_Hausdorff_distance = std::max(Hausdorff_distA, Hausdorff_distB);
	//std::cout << "Hausdorff distance from the original model to the resulting model: " << Hausdorff_distA << std::endl;
	//std::cout << "Hausdorff distance from the resulting model to the original model: " << Hausdorff_distB << std::endl;
	std::cout << "symmetric Hausdorff distance: " << symmetric_Hausdorff_distance << std::endl;

}
