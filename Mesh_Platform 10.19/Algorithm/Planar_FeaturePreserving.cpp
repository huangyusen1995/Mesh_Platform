#include "Planar_FeaturePreserving.h"

#include <stack>

//#include <Eigen/Dense>
#include ".\3rd_party\eigen\Eigen\Dense"
//#include <Eigen/Sparse>
#include ".\3rd_party\eigen\Eigen\Sparse"

namespace easy3d {

	//��������
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
		SurfaceMesh *mesh, SurfaceMesh::FaceProperty<int> id,                //idΪ�ָ��ǩ������
		float angle_threshold, float area_threshold, float fitting_threshold)
	{
		//��ʼ��id����
		mesh->update_face_normals();                                         //�����淨����
		auto is_degenerate = mesh->add_face_property<bool>("f:SurfaceMeshEnumerator:is_degenerate", false);//�������������飬�Ƿ��˻�

		int num_degenerate = 0;                                       //�˻�����
		for (auto f : mesh->faces()) {                                //����ÿ����ָ�= -1������Ѿ��˻�����
			id[f] = -1;
			if (mesh->is_degenerate(f)) {                            //������f�Ƿ��˻����ߵļн�С��ĳ��ֵ��
				is_degenerate[f] = true;
				++num_degenerate;
			}
		}


//*************************��һ�׶Σ���������������id�����****************//
		int cur_id = 0;
		for (auto f : mesh->faces()) {
			if (/*!is_degenerate[f] &&*/ id[f] == -1) {                          //����ÿ���棬���δ�˻� && δ�ָ�
				propagate_planar_component(mesh, id, f, cur_id, angle_threshold);//����f���������ID��cur_id��������������ֱ������
				cur_id++;                                                        //�������ID ++
			}
		}
		//���붨��һ������ͼ���ڽӾ���Ӧ����ʲô����
		int now_id = cur_id;
		std::cout << "The angle threshold theta:" << angle_threshold << "\n";
		std::cout << "The number of proxies after region growing:" << now_id << "\n";
//*************************�ڶ��׶Σ������ں�*******************************//

		/////////////////////һ����ʼ��:��������������������/////////////////////////
		std::vector<vec3> proxy_normals(cur_id);
		std::vector<int> proxy_facecount(cur_id);
		std::vector<float> proxy_areas(cur_id);
		std::vector<std::vector<int>> proxy_nearproxies(cur_id);
		std::vector<std::vector<SurfaceMesh::Face>> proxy_faces(cur_id);

		SurfaceMesh::FaceProperty<vec3> fnormal_ = mesh->get_face_property<vec3>("f:normal");
		SurfaceMesh::VertexProperty<vec3> vpoint_ = mesh->get_vertex_property<vec3>("v:point");

		for (auto f : mesh->faces()) //����ÿ��������ķ������ʹ����������������
		{
			proxy_normals[id[f]] += fnormal_[f];             //�����ܷ�����
			proxy_facecount[id[f]]++;                        //�������������
			proxy_faces[id[f]].push_back(f);                 //�������������f

			assert(mesh->valence(f) == 3);
			auto fv = mesh->vertices(f);
			const auto &p0 = mesh->position(*fv);
			const auto &p1 = mesh->position(*(++fv));
			const auto &p2 = mesh->position(*(++fv));
			proxy_areas[id[f]] += geom::triangle_area(p0, p1, p2);//���������
		}
		for (int i = 0; i < proxy_normals.size(); i++) proxy_normals[i] = proxy_normals[i] / proxy_facecount[i];//����ƽ��������

		/////////////////////////////������ͼ:�������ڽӹ�ϵ///////////////////////////
		Eigen::Matrix<bool, -1, -1> matA;
		matA.resize(cur_id, cur_id);
		matA.setZero();
		for (int i = 0; i < proxy_faces.size(); ++i)   //����ÿ������
		{
			for (int j = 0; j < proxy_faces[i].size(); ++j)//����ÿ����
			{
				for (auto h : mesh->halfedges(proxy_faces[i][j])) {   //����proxy_faces[i][j]��İ���ҵ���Աߵ�������cur��
					auto cur = mesh->face(mesh->opposite(h));         //������cur��
					if (id[cur] != i)
					{
						matA(i, id[cur]) = 1;
						matA(id[cur], i) = 1;
						//triplets.emplace_back(i, id[cur], 1);//������Ĵ����ǵ�ǰ�Ĵ�������
					}

				}
			}
		}

		///////////////////////////////���������ں�///////////////////////
		for (int i = 0; i < matA.rows(); ++i)
		{
			std::vector<SurfaceMesh::Vertex> proxy_vertices;
			float min_fitting_distance = 1000.0;
			int target_proxy = -1;

			if (proxy_areas[i] > area_threshold)
				continue;

			else
				proxy_vertices.clear();
			    for (int j = 0; j < proxy_faces[i].size(); ++j) //����ÿ�������ڵ�������ɾ���ظ��Ķ���
			    {
				    for (auto vv : mesh->vertices(proxy_faces[i][j])) //������Ķ����������
				    {
					    proxy_vertices.push_back(vv);
				    }
			    }
			    sort(proxy_vertices.begin(), proxy_vertices.end());
			    proxy_vertices.erase(unique(proxy_vertices.begin(), proxy_vertices.end()), proxy_vertices.end());//ɾ���ظ��Ķ���

			    for (int j = 0; j < matA.cols(); ++j)//�����ҷ���ֵ==���ڴ���
			    {
				//iΪ��ǰ����
				//jΪ�ڽ�����
				    if (matA(i, j) == 1 && i != j)
				    {
					    float dist = 0.0;
					    vec3 nn = proxy_normals[j];                                         //j����ķ�����
					    vec3 aa = vpoint_[mesh->target(mesh->halfedge(proxy_faces[j][0]))]; //j���������ϵ�һ��
					    for (int p = 0; p < proxy_vertices.size(); p++)
					    {
						    vec3 ap = vpoint_[proxy_vertices[p]] - aa;               //i������һ���ȥ����j������һ��
					        
						    dist += (abs(dot(nn, ap)) / sqrt(nn[0] * nn[0] + nn[1] * nn[1] + nn[2] * nn[2]));
					    }

					    if (dist < min_fitting_distance)
					    {
						    min_fitting_distance = dist;
						    target_proxy = j;//�˴���ΪĿ���ںϵ��ڽ�����,Ҫ��i����target_proxy�ڣ�����������
					    }
				    }
			    }

			    if (min_fitting_distance < fitting_threshold && target_proxy != -1)
			    {
				//���´�������proxy_normals������proxy_nearproxies������proxy_faces������id��

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

				    for (int m = 0; m < (int)proxy_faces[i].size(); m++)//����proxy_faces������id��
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
				    for (int m = 0; m < (int)proxy_faces[i].size(); m++)//����proxy_faces������id��
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
		//�����˻���������
		if (num_degenerate > 0) { // propagate the planar partition to degenerate faces ����ƽ��������˻����ϣ��˴����δ���ָ�����򣬸������ڷָ�����
			LOG(WARNING) << "model has " << num_degenerate << " degenerate faces" << std::endl;
			int num_propagated = 0;
			do {
				num_propagated = 0;
				for (auto e : mesh->edges()) {                   //����ÿ���ߣ��ҵ��������棬���f0���˻���δ�ָ���� && f1δ�˻�����f1�ķָ���������f0
					auto f0 = mesh->face(mesh->halfedge(e, 0));  //���ر�e�ĵ�i����ߣ�i������0��1��
					auto f1 = mesh->face(mesh->halfedge(e, 1));  //f0��f1�Ǳ�e����������
					if (f0.is_valid() && f1.is_valid()) {
						if (is_degenerate[f0] && id[f0] == -1 && !is_degenerate[f1]) {
							id[f0] = id[f1];                     //���˻������ε��������������ں�
							++num_propagated;
						}
					}
				}
			} while (num_propagated > 0);
		}

		mesh->remove_face_property(is_degenerate);
		//std::cout << cur_id << std::endl;
		return cur_id;            //��������ĸ���
	}


}