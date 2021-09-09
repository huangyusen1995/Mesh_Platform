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


//原始模型上的顶点到当前模型上最近的面的平均距离
void SurfaceMesh_error::mean_error(SurfaceMesh *original_mesh)
{
	float total_error = 0.0;

	for(auto v: original_mesh->vertices())
	{
		float min_dist = 100.0;
		
		for(auto f: mesh_->faces())
		{
			vec3 ap = original_mesh->position(v) - mesh_->position(mesh_->target(mesh_->halfedge(f)));  //original_mesh中v点减去result_mesh内一点
			vec3 nn = fnormal[f];                                                                                          //面法向量方法待定
			float dist = (abs(dot(nn, ap)) / sqrt(nn[0] * nn[0] + nn[1] * nn[1] + nn[2] * nn[2]));                        //点面距
			if (dist < min_dist) min_dist = dist;
		}
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
			vec3 ap = original_mesh->position(v) - mesh_->position(mesh_->target(mesh_->halfedge(f)));  //original_mesh中v点减去result_mesh内一点
			vec3 nn = fnormal[f];                                                                                         
			float dist = (abs(dot(nn, ap)) / sqrt(nn[0] * nn[0] + nn[1] * nn[1] + nn[2] * nn[2]));  //点面距
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
	//从原始模型到结果模型的单向Hausdorff距离
	float distA = 0.0;
	float Hausdorff_distA = 0.0;
	for (auto v : original_mesh->vertices())
	{
		float min_distA = 100.0;
		for (auto f : mesh_->faces())
		{
			//original_mesh中v点减去result_mesh内一点
			vec3 ap = original_mesh->position(v) - mesh_->position(mesh_->target(mesh_->halfedge(f))); 
			//两点距离
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

	//从结果模型到原始模型的单向Hausdorff距离
	float distB = 0.0;
	float Hausdorff_distB = 0.0;
	for (auto vb : mesh_->vertices())
	{
		float min_distB = 100.0;
		for (auto fb : original_mesh->faces())
		{
			//original_mesh中v点减去result_mesh内一点
			vec3 bp = mesh_->position(vb) - original_mesh->position(original_mesh->target(original_mesh->halfedge(fb)));
			//两点距离
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
