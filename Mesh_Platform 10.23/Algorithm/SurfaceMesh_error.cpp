#include "SurfaceMesh_error.h"

using namespace easy3d;

SurfaceMesh_error::SurfaceMesh_error(SurfaceMesh *mesh) : mesh_(mesh)
{
	mesh_->update_face_normals();
	fnormal = mesh_->get_face_property<vec3>("f:normal");
	m_b = -0.005;
	//float ss = 0.0;  //单位化:点坐标除以L2
   
}



SurfaceMesh_error::~SurfaceMesh_error()
{
}


//原始模型上的顶点到当前模型上最近的面的平均距离
void SurfaceMesh_error::mean_error(SurfaceMesh *original_mesh)
{
	auto min_distance = original_mesh->vertex_property<double>("v:min_distance", 0.0);
	//auto min_distance = original_mesh->vertex_property<float>("v:min_distance", 0.0);
	float total_error = 0.0;
	float mean_error = 0.0;

	////单位化:点坐标除以最远点轴坐标的绝对值:float unitize
	//const Box3 &box = original_mesh->bounding_box(); 
	//float maxcrood0 = abs((std::min(box.min_coord(0), std::min(box.min_coord(1), box.min_coord(2)))));	
	//float maxcrood1 = abs((std::max(box.max_coord(0), std::max(box.max_coord(1), box.max_coord(2)))));
	//float unitize = std::max(maxcrood0, maxcrood1);
	//单位化:点坐标除以box对角线长度
	//const Box3 &box = mesh_->bounding_box();
	//float unitize = box.diagonal();
	//for (auto v : mesh_->vertices())
	//{
	//	mesh_->position(v) /= unitize;
	//}



	/*float ss = 0.0;
	for (auto v : mesh_->vertices())
	{
		 ss += (mesh_->position(v).length2());
	}
	ss = sqrt(ss)/mesh_->n_vertices();*/

	for(auto v: original_mesh->vertices())
	{
		float min_dist = 100.0;
		
		for(auto f: mesh_->faces())
		{
			vec3 &avp = original_mesh->position(v);
			//float avp_length = original_mesh->position(v).length();
			//vec3& avp_norm = avp / avp_length;
			//vec3& avp_norm1 = avp / unitize;
			
			vec3 &afp= mesh_->position(mesh_->target(mesh_->halfedge(f)));  //original_mesh中v点减去result_mesh内一点
			//float afp_length = mesh_->position(mesh_->target(mesh_->halfedge(f))).length();
			//vec3 &afp_norm = afp / afp_length;
			vec3 &ap = avp - afp;
			//vec3 &ap = avp_norm - afp_norm;
			vec3 &nn = fnormal[f]; 
			//std::cout << "ap:" << ap << std::endl;//面法向量方法待定
			//std::cout << "nn:" << nn << std::endl;
			float dist = (abs(dot(ap, nn)) / sqrt(nn[0] * nn[0] + nn[1] * nn[1] + nn[2] * nn[2]));                        //点面距
			if (dist < min_dist) min_dist = dist;
		}
		double emphasized_value = log((m_b - min_dist) / m_b) / log((m_b - 1.0) / m_b);
		//std::cout << v << ":" << emphasized_value << std::endl;
		min_distance[v] = min_dist;
		//min_distance[v] = emphasized_value;
		//std::cout << "min distance:" << min_dist << std::endl;
		total_error += min_dist;
		mean_error += emphasized_value;

	}
	
	int vertices_num = original_mesh->n_vertices();
	if (vertices_num != 0)
	{
		total_error /= vertices_num;
		mean_error /= vertices_num;

		std::cout << "vertices_num = " << vertices_num << std::endl;
		std::cout << "mean total error = " << total_error << std::endl;
		std::cout << "mean emphasized_value error = " << mean_error << std::endl;
	}
	else
	{
		std::cout << "vertices number:0   It's impossible to calculate mean error" << std::endl;
	}
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
	int vertices_num= original_mesh->n_vertices();
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
	//float distB = 0.0;
	//float Hausdorff_distB = 0.0;
	//for (auto vb : mesh_->vertices())
	//{
	//	float min_distB = 100.0;
	//	for (auto fb : original_mesh->faces())
	//	{
	//		//original_mesh中v点减去result_mesh内一点
	//		vec3 bp = mesh_->position(vb) - original_mesh->position(original_mesh->target(original_mesh->halfedge(fb)));
	//		//两点距离
	//		distB = sqrt(bp[0] * bp[0] + bp[1] * bp[1] + bp[2] * bp[2]);
	//		if (distB < min_distB)
	//		{
	//			min_distB = distB;
	//		}
	//	}
	//	if (min_distB > Hausdorff_distB)
	//	{
	//		Hausdorff_distB = min_distB;
	//	}
	//}
	//
	//float symmetric_Hausdorff_distance = std::max(Hausdorff_distA, Hausdorff_distB);
	std::cout << "Hausdorff distance from the original model to the resulting model: " << Hausdorff_distA << std::endl;
	//std::cout << "Hausdorff distance from the resulting model to the original model: " << Hausdorff_distB << std::endl;
	//std::cout << "symmetric Hausdorff distance: " << symmetric_Hausdorff_distance << std::endl;

}

void SurfaceMesh_error::SAMD_mean_error(SurfaceMesh *original_mesh)
{
	float SAMD_mean_error = 0.0;
	//从原始模型到结果模型的平均距离
	float distA = 0.0;
	float mean_distA = 0.0;
	for (auto v : original_mesh->vertices())
	{
		float min_distA = 100.0;
		vec3 &ap1 = original_mesh->position(v);
		for (auto vb : mesh_->vertices())
		{
			//original_mesh中v点减去result_mesh内一点
			//vec3 ap = original_mesh->position(v) - mesh_->position(mesh_->target(mesh_->halfedge(f)));
			vec3 &ap2= mesh_->position(vb);
			distA = distance(ap1,ap2);
			//std::cout << "distA:" << distA << std::endl;
			if (distA < min_distA)
			{
				min_distA = distA;
			}
		}
		mean_distA += min_distA;
	}
	mean_distA/= original_mesh->n_vertices();

	//从结果模型到原始模型的距离
	float distB = 0.0;
	float mean_distB = 0.0;
	for (auto vb : mesh_->vertices())
	{
		float min_distB = 100.0;
		vec3 &bp1 = mesh_->position(vb);
		//for (auto fb : original_mesh->faces())
		for(auto v: original_mesh->vertices())
		{
			//vec3 bp = mesh_->position(vb) - original_mesh->position(original_mesh->target(original_mesh->halfedge(fb)));
			////两点距离
			//distB = sqrt(bp[0] * bp[0] + bp[1] * bp[1] + bp[2] * bp[2]);
			//vec3 bp = mesh_->position(vb) - original_mesh->position(original_mesh->target(original_mesh->halfedge(fb)));  //original_mesh中v点减去result_mesh内一点
			
			vec3 &bp2=original_mesh->position(v);

			//vec3 nn = original_mesh->compute_face_normal(fb);
			//std::cout << "ap:" << ap << std::endl;//面法向量方法待定
			//std::cout << "nn:" << nn << std::endl;
			//float distB = (abs(dot(bp, nn)) / sqrt(nn[0] * nn[0] + nn[1] * nn[1] + nn[2] * nn[2]));
			//distB = sqrt(bp[0] * bp[0] + bp[1] * bp[1] + bp[2] * bp[2]);
			distB = distance(bp1, bp2);
			if (distB < min_distB)
			{
				min_distB = distB;
			}
		}
		mean_distB += min_distB;
	}
	mean_distB /= mesh_->n_vertices();

	SAMD_mean_error = std::max(mean_distA, mean_distB);
	std::cout << "SAMD mean_distA:" << mean_distA << std::endl;
	std::cout << "SAMD mean_distB:" << mean_distB << std::endl;
	std::cout << "SAMD mean error:" << SAMD_mean_error << std::endl;

}