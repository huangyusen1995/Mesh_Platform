#include "Planar_FeaturePreserving.h"

#include <stack>

//#include <Eigen/Dense>
#include ".\3rd_party\eigen\Eigen\Dense"
//#include <Eigen/Sparse>
#include ".\3rd_party\eigen\Eigen\Sparse"

namespace easy3d {

	//区域生长
	void Planar_FeaturePreserving::propagate_planar_component(SurfaceMesh *mesh,
		SurfaceMesh::FaceProperty<int> id,
		SurfaceMesh::Face seed, int cur_id,
		float angle_threshold
	) {
		auto fnormals = mesh->get_face_property<vec3>("f:normal");
		auto is_degenerate = mesh->get_face_property<bool>("f:SurfaceMeshEnumerator:is_degenerate");

		std::stack<SurfaceMesh::Face> stack;
		stack.push(seed);

		while (!stack.empty()) {
			SurfaceMesh::Face top = stack.top();
			stack.pop();
			if (id[top] == -1) {
				id[top] = cur_id;
				const vec3 &n_top = fnormals[top];
				for (auto h : mesh->halfedges(top)) {
					auto cur = mesh->face(mesh->opposite(h));
					if (cur.is_valid() && id[cur] == -1 && !is_degenerate[cur]) {
						const vec3 &n_cur = fnormals[cur];
						auto angle = geom::angle(n_top, n_cur); // in [-pi, pi]
						angle = rad2deg(std::abs(angle));
						if (std::abs(angle) < angle_threshold)
							stack.push(cur);
					}
				}
			}
		}
	}


	int Planar_FeaturePreserving::enumerate_planar_Featurepreserving(
		SurfaceMesh *mesh, SurfaceMesh::FaceProperty<int> id,                //id为分割标签的数组
		float angle_threshold, float area_threshold, float fitting_threshold)
	{
		//初始化id数组
		mesh->update_face_normals();                                         //更新面法向量
		auto is_degenerate = mesh->add_face_property<bool>("f:SurfaceMeshEnumerator:is_degenerate", false);//加入面属性数组，是否退化

		int num_degenerate = 0;                                       //退化面数
		for (auto f : mesh->faces()) {                                //设置每个面分割= -1；标记已经退化的面
			id[f] = -1;
			if (mesh->is_degenerate(f)) {                            //返回面f是否退化（边的夹角小于某阈值）
				is_degenerate[f] = true;
				++num_degenerate;
			}
		}


//*************************第一阶段：进行区域生长：id的填充****************//
		int cur_id = 0;
		for (auto f : mesh->faces()) {
			if (/*!is_degenerate[f] &&*/ id[f] == -1) {                          //对于每个面，如果未退化 && 未分割
				propagate_planar_component(mesh, id, f, cur_id, angle_threshold);//对面f和面的区域ID：cur_id进行区域增长，直到结束
				cur_id++;                                                        //面的区域ID ++
			}
		}
		//我想定义一个无向图的邻接矩阵，应该用什么类型
		int now_id = cur_id;
		std::cout << "The angle threshold theta:" << angle_threshold << "\n";
		std::cout << "The number of proxies after region growing:" << now_id << "\n";
//*************************第二阶段：区域融合*******************************//

		/////////////////////一、初始化:代理法向量、代理含三角面/////////////////////////
		std::vector<vec3> proxy_normals(cur_id);
		std::vector<int> proxy_facecount(cur_id);
		std::vector<float> proxy_areas(cur_id);
		std::vector<std::vector<int>> proxy_nearproxies(cur_id);
		std::vector<std::vector<SurfaceMesh::Face>> proxy_faces(cur_id);

		SurfaceMesh::FaceProperty<vec3> fnormal_ = mesh->get_face_property<vec3>("f:normal");
		SurfaceMesh::VertexProperty<vec3> vpoint_ = mesh->get_vertex_property<vec3>("v:point");

		for (auto f : mesh->faces()) //计算每个代理面的法向量和代理面的三角面数。
		{
			proxy_normals[id[f]] += fnormal_[f];             //代理总法向量
			proxy_facecount[id[f]]++;                        //代理含三角面计数
			proxy_faces[id[f]].push_back(f);                 //代理包含三角面f

			assert(mesh->valence(f) == 3);
			auto fv = mesh->vertices(f);
			const auto &p0 = mesh->position(*fv);
			const auto &p1 = mesh->position(*(++fv));
			const auto &p2 = mesh->position(*(++fv));
			proxy_areas[id[f]] += geom::triangle_area(p0, p1, p2);//代理总面积
		}
		for (int i = 0; i < proxy_normals.size(); i++) proxy_normals[i] = proxy_normals[i] / proxy_facecount[i];//代理平均法向量

		/////////////////////////////二、建图:代理面邻接关系///////////////////////////
		Eigen::Matrix<bool, -1, -1> matA;
		matA.resize(cur_id, cur_id);
		matA.setZero();
		for (int i = 0; i < proxy_faces.size(); ++i)   //对于每个代理
		{
			for (int j = 0; j < proxy_faces[i].size(); ++j)//对于每个面
			{
				for (auto h : mesh->halfedges(proxy_faces[i][j])) {   //对于proxy_faces[i][j]面的半边找到其对边的邻域面cur：
					auto cur = mesh->face(mesh->opposite(h));         //邻域面cur：
					if (id[cur] != i)
					{
						matA(i, id[cur]) = 1;
						matA(id[cur], i) = 1;
						//triplets.emplace_back(i, id[cur], 1);//邻域面的代理不是当前的代理，加入
					}

				}
			}
		}

		///////////////////////////////三：区域融合///////////////////////
		for (int i = 0; i < matA.rows(); ++i)
		{
			std::vector<SurfaceMesh::Vertex> proxy_vertices;
			float min_fitting_distance = 1000.0;
			int target_proxy = -1;

			if (proxy_areas[i] > area_threshold)
				continue;

			else
				proxy_vertices.clear();
			    for (int j = 0; j < proxy_faces[i].size(); ++j) //对于每个代理内的三角面删除重复的顶点
			    {
				    for (auto vv : mesh->vertices(proxy_faces[i][j])) //三角面的顶点存入数组
				    {
					    proxy_vertices.push_back(vv);
				    }
			    }
			    sort(proxy_vertices.begin(), proxy_vertices.end());
			    proxy_vertices.erase(unique(proxy_vertices.begin(), proxy_vertices.end()), proxy_vertices.end());//删除重复的顶点

			    for (int j = 0; j < matA.cols(); ++j)//迭代找非零值==相邻代理
			    {
				//i为当前代理
				//j为邻近代理
				    if (matA(i, j) == 1 && i != j)
				    {
					    float dist = 0.0;
					    vec3 nn = proxy_normals[j];                                         //j代理的法向量
					    vec3 aa = vpoint_[mesh->target(mesh->halfedge(proxy_faces[j][0]))]; //j代理内面上的一点
					    for (int p = 0; p < proxy_vertices.size(); p++)
					    {
						    vec3 ap = vpoint_[proxy_vertices[p]] - aa;               //i代理内一点减去邻域j代理内一点
					        
						    dist += (abs(dot(nn, ap)) / sqrt(nn[0] * nn[0] + nn[1] * nn[1] + nn[2] * nn[2]));
					    }

					    if (dist < min_fitting_distance)
					    {
						    min_fitting_distance = dist;
						    target_proxy = j;//此代理为目标融合的邻近代理,要把i融入target_proxy内！！！！！！
					    }
				    }
			    }

			    if (min_fitting_distance < fitting_threshold && target_proxy != -1)
			    {
				//更新代理法向量proxy_normals？更新proxy_nearproxies！更新proxy_faces！更新id！

				    for (int j = 0; j < matA.cols(); ++j)
				    {
					    if (matA(i, j) == 1 && i != j)
					    {
						    matA(j, target_proxy) = 1;
						    matA(target_proxy, j) = 1;
						    matA(i, j) = 0;
						    matA(j, i) = 0;
						    matA(i, i) = 0;

					    }
				    }

				    for (int m = 0; m < (int)proxy_faces[i].size(); m++)//更新proxy_faces！更新id！
				    {
					    proxy_faces[target_proxy].push_back(proxy_faces[i][m]);
					    id[proxy_faces[i][m]] = target_proxy;
				    }
				    proxy_faces[i].clear();

				    proxy_areas[target_proxy] += proxy_areas[i];
			    }
			    else
			    {
				    for (int j = 0; j < matA.cols(); ++j)
				    {
					    if (matA(i, j) == 1 && i != j)
					    {
						    matA(i, j) = 0;
						    matA(j, i) = 0;
						    matA(i, i) = 0;
					    }

				    }
				    for (int m = 0; m < (int)proxy_faces[i].size(); m++)//更新proxy_faces！更新id！
				    {
					    id[proxy_faces[i][m]] = -1;
				    }
				    proxy_faces[i].clear();
			    }
			    now_id--;
			    //cur_id--;
		}
		std::cout << "The area threshold:" << area_threshold << "   ";
		std::cout << "The fitting threshold:" << fitting_threshold << "\n";
		std::cout << "The number of proxies after region fusion:" << now_id << "\n";
		//后处理退化的三角形
		if (num_degenerate > 0) { // propagate the planar partition to degenerate faces 传播平面分区到退化面上：此处检测未被分割的区域，赋给相邻分割区域
			LOG(WARNING) << "model has " << num_degenerate << " degenerate faces" << std::endl;
			int num_propagated = 0;
			do {
				num_propagated = 0;
				for (auto e : mesh->edges()) {                   //对于每个边，找到两个邻面，如果f0已退化、未分割代理 && f1未退化：把f1的分割区域结果给f0
					auto f0 = mesh->face(mesh->halfedge(e, 0));  //返回边e的第i个半边，i必须是0或1。
					auto f1 = mesh->face(mesh->halfedge(e, 1));  //f0、f1是边e的两个邻面
					if (f0.is_valid() && f1.is_valid()) {
						if (is_degenerate[f0] && id[f0] == -1 && !is_degenerate[f1]) {
							id[f0] = id[f1];                     //把退化三角形的区域代理跟邻域融合
							++num_propagated;
						}
					}
				}
			} while (num_propagated > 0);
		}

		mesh->remove_face_property(is_degenerate);
		//std::cout << cur_id << std::endl;
		return cur_id;            //返回区域的个数
	}


}