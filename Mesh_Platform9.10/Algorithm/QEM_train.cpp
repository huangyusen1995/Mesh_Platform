
#include <easy3d/core/surface_mesh.h>
#include <easy3d/fileio/surface_mesh_io.h>
#include <easy3d/fileio/resources.h>
//#include <.\3rd_party\eigen\Eigen\src\Core\Matrix.h>
//#include <Eigen/Dense>
#include ".\3rd_party\eigen\Eigen\Dense"
//#include <Eigen/Sparse>
#include ".\3rd_party\eigen\Eigen\Sparse"
//这个文件进行了网格塌陷QEM练习操作

#include "QEM_trian.h"
#include "Planar_FeaturePreserving.h"
#include <cfloat>
#include <iterator> // for back_inserter on Windows
#define REAL double

using namespace easy3d;

namespace easy3d {

	SurfaceMeshQEM::SurfaceMeshQEM(SurfaceMesh *mesh)  //SurfaceMeshSimplification类的初始化构造
		: mesh_(mesh), initialized_(false), queue_(nullptr) {

		aspect_ratio_ = 0;            //纵横比
		edge_length_ = 0;
		normal_deviation_ = 0;        //法线偏差
		// get properties 找到点的属性，为数组
		vpoint_ = mesh_->get_vertex_property<vec3>("v:point");
		
		// compute face normals 找到面的属性，为数组
		mesh_->update_face_normals();
		fnormal_ = mesh_->get_face_property<vec3>("f:normal");
		
	}

	//-----------------------------------------------------------------------------

	SurfaceMeshQEM::~SurfaceMeshQEM() {
	}

	//-----------------------------------------------------------------------------

	void SurfaceMeshQEM::initialize(int cur_id, float aspect_ratio , float edge_length, float normal_deviation ) {
		if (!mesh_->is_triangle_mesh())   //如果mesh不是三角网格，退出
			return;

		// store parameters 存储参数
		aspect_ratio_ = aspect_ratio;
		normal_deviation_ = normal_deviation / 180.0 * M_PI;
		edge_length_ = edge_length;
		if (normal_deviation_ > 0.0)
			normal_cone_ = mesh_->face_property<NormalCone>("f:normalCone");
		else
			mesh_->remove_face_property(normal_cone_);
		
		// initialize normal cones 初始化法线锥？？？？
		if (normal_deviation_) {
			for (auto f : mesh_->faces()) {
				normal_cone_[f] = NormalCone(fnormal_[f]);
			}
		}
		
		// add quadric property对每个点加入二次误差的属性
		vquadric_ = mesh_->add_vertex_property<Quadric>("v:quadric");
		//计算代理
		//用平均值计算各个代理面的法向量
		std::vector<vec3> proxy_normals(cur_id);
		std::vector<int> proxy_facecount(cur_id);

		//if (mesh_->get_face_property<int>("f:planar_partition"))
		if (cur_id != 0)
		{
			std::cout << "use LML.QEM with proxy" << "\n";
			cur_id = cur_id;
			planar_segments = mesh_->get_face_property<int>("f:planar_partition");

			for (auto f : mesh_->faces()) //计算每个代理面的法向量和代理面的三角面数。
			{
				if (planar_segments[f] != -1)
				{
					proxy_normals[planar_segments[f]] += fnormal_[f];
					proxy_facecount[planar_segments[f]]++;
				}

			}
			for (int i = 0; i < proxy_normals.size(); i++) proxy_normals[i] = proxy_normals[i] / proxy_facecount[i];

			//此处为李明磊的QEM计算：点中判断加入代理面的二次误差
		    for (auto v : mesh_->vertices()) {          //对于每个顶点操作：
			    vquadric_[v].clear();
			    SurfaceMesh::Face ff = mesh_->face(mesh_->out_halfedge(v)); //点的其中一个邻面

			    bool neighbor_is_proxy = TRUE;
			    if (!mesh_->is_isolated(v)) {               //is_isolated返回v是否被隔离，即不与任何面发生关联
				    for (auto f : mesh_->faces(v))          //对于点的每个邻域面判断是否都属于代理：
				    {       
					    if (planar_segments[ff] != planar_segments[f] || planar_segments[f] == -1)
					    {
						    neighbor_is_proxy = FALSE;
						    break;
					    }
				    }
				    if(neighbor_is_proxy)                  //都属于代理，则用代理面计算二次误差
				    {
					    vquadric_[v] = Quadric(proxy_normals[planar_segments[ff]], vpoint_[v]);
				    }
				    else                                  //不都属于代理，用原始QEM
				    {
					    for (auto f : mesh_->faces(v)) vquadric_[v] += Quadric(fnormal_[f], vpoint_[v]);//fnormal_[f]为面f法向量；vpoint_[v]为点v坐标 %%%%%%
				    }
				
			    }   //vquadric_[v]为点v的邻域面的平方距离
		    }
		}
		else
		{
			//std::cout << "use default region grow paras: 2.0f, 0.5f, 0.1f";
			//const std::string partition_name = "f:planar_partition";        //代理属性名
			//planar_segments = mesh_->face_property<int>(partition_name, -1);//对每个面加入代理属性
			//int cur_id = Planar_FeaturePreserving::enumerate_planar_Featurepreserving(mesh_, planar_segments, 2.0f, 0.5f, 0.1f);
		    std::cout << "use G.H.QEM without proxy" << "\n";
			//此处为原始G.H.的QEM计算。
		    //initialize quadrics 初始化点的二次误差
			for (auto v : mesh_->vertices()) {          //对于每个顶点操作：
				vquadric_[v].clear();

				if (!mesh_->is_isolated(v)) {               //is_isolated返回v是否被隔离，即不与任何面发生关联
					for (auto f : mesh_->faces(v)) {        //对于点邻近的每个面操作：
						vquadric_[v] += Quadric(fnormal_[f], vpoint_[v]);//fnormal_[f]为面f法向量；vpoint_[v]为点v坐标 %%%%%%
					}
				}   //vquadric_[v]为点v的邻域面的平方距离
			}
		}


		
	
		
		//此处为李明磊的QEM计算：点中判断加入代理面的二次误差
		//for (auto v : mesh_->vertices()) {          //对于每个顶点操作：
		//	vquadric_[v].clear();
		//	SurfaceMesh::Face ff = mesh_->face(mesh_->out_halfedge(v)); //点的其中一个邻面

		//	bool neighbor_is_proxy = TRUE;
		//	if (!mesh_->is_isolated(v)) {               //is_isolated返回v是否被隔离，即不与任何面发生关联
		//		for (auto f : mesh_->faces(v))          //对于点的每个邻域面判断是否都属于代理：
		//		{       
		//			if (planar_segments[ff] != planar_segments[f] || planar_segments[f] == -1)
		//			{
		//				neighbor_is_proxy = FALSE;
		//				break;
		//			}
		//		}
		//		if(neighbor_is_proxy)                  //都属于代理，则用代理面计算二次误差
		//		{
		//			vquadric_[v] = Quadric(proxy_normals[planar_segments[ff]], vpoint_[v]);
		//		}
		//		else                                  //不都属于代理，用原始QEM
		//		{
		//			for (auto f : mesh_->faces(v)) vquadric_[v] += Quadric(fnormal_[f], vpoint_[v]);//fnormal_[f]为面f法向量；vpoint_[v]为点v坐标 %%%%%%
		//		}
		//		
		//	}   //vquadric_[v]为点v的邻域面的平方距离
		//}
		
		//cur_id = 0;
		initialized_ = true;
		
	}

	//-----------------------------------------------------------------------------

	void SurfaceMeshQEM::simplify(unsigned int n_vertices) {    //输入最终想要的点的数量
		if (!mesh_->is_triangle_mesh()) {                                 // 如果mesh是三角网格才操作
			std::cerr << "Not a triangle mesh!" << std::endl;
			return;
		}


		unsigned int nv(mesh_->n_vertices());            //nv为最初的点总数量；
		
		std::vector<SurfaceMesh::Vertex> one_ring;       //点的一环邻域点
		std::vector<SurfaceMesh::Vertex>::iterator or_it, or_end;//一环邻域点的迭代器！！@
		SurfaceMesh::Halfedge h;                         //半边h
		SurfaceMesh::Vertex v;                           //点v
		vec3 newv;
		// add properties for priority queue  为优先队列添加属性
		vpriority_ = mesh_->add_vertex_property<float>("v:prio"); //每个点的优先级。小数数组
		heap_pos_ = mesh_->add_vertex_property<int>("v:heap");    //每个点的堆中位置。整数数组
		vtarget_ = mesh_->add_vertex_property<SurfaceMesh::Halfedge>("v:target");//每个点的出边？半边数组
		
		//new_vertices = mesh_->add_vertex_property<vec3>("v:new_ver");

		// build priority queue建立优先队列
		HeapInterface hi(vpriority_, heap_pos_);        //每个点的优先级和每个点的堆中位置加入堆接口。
		queue_ = new PriorityQueue(hi);                 //新的堆队列
		queue_->reserve(mesh_->n_vertices());           //为最初的点总数量 保留空间
		for (auto v : mesh_->vertices()) {              //对每个点操作：！！
			queue_->reset_heap_position(v);                 //将v点堆位置重置为-1(不在堆中)：pos_[v] = -1
			enqueue_vertex(v);                              //点的每个出边都在队列中有代价vpriority_和标记vtarget_
		}
		
		while (nv > n_vertices && !queue_->empty()) {
			// get 1st element
			v = queue_->front();      //第一个条目出一个顶点
			queue_->pop_front();      //删除第一个条目v
			h = vtarget_[v];          //得到v点对应的一个塌陷边h，此时的h为min_h！！！@！
			
			CollapseData cd(mesh_, h); //点v的各个数据
			newv = compute_new_v(cd);

			// check this (again)       再次检查h是否可以塌陷
			if (!mesh_->is_collapse_ok(h))
				continue;

			// store one-ring
			one_ring.clear();
			for (auto vv : mesh_->vertices(cd.v0)) {   //返回顶点v0周围的所有顶点
				one_ring.push_back(vv);                //加入 one_ring数组
			}

			// perform collapse 执行塌陷
			this->collapse_qem_LML(h, newv);
			//mesh_->collapse_qem_LML(h, newv);
			//mesh_->collapse(h);原easy3d的塌陷，将中心点v0选择最小边h（v0v1）塌陷至v1。

			--nv;
			
			//if (nv % 1000 == 0) std::cerr << nv << "\r";

			// postprocessing, e.g., update quadrics后处理，更新二次误差，删除队列中的顶点
			postprocess_collapse(cd);

			// update queue            更新队列
			for (or_it = one_ring.begin(), or_end = one_ring.end(); or_it != or_end; //一环邻域点的迭代器or_it
				++or_it)
				enqueue_vertex(*or_it);                   //对每个已经处理的点v来说，处理它的一环邻域点的二次误差
		
		}
		
		// clean up 全部塌陷完，清理
		delete queue_;
		mesh_->collect_garbage();
		mesh_->remove_vertex_property(vpriority_);
		mesh_->remove_vertex_property(heap_pos_);
		mesh_->remove_vertex_property(vtarget_);
		//mesh_->remove_vertex_property(new_vertices);
		// remove added properties
		mesh_->remove_vertex_property(vquadric_); 
		mesh_->remove_face_property(normal_cone_);
		mesh_->remove_face_property(face_points_); 
		mesh_->remove_face_property(planar_segments);
	}

	//-----------------------------------------------------------------------------

	void SurfaceMeshQEM::enqueue_vertex(SurfaceMesh::Vertex v) {  //对顶点v0在队列中的值进行排序整理
		float prio, min_prio(FLT_MAX);
		SurfaceMesh::Halfedge min_h;
		vec3 new_v;

		// find best out-going halfedge找到最好的外向半边
		for (auto h : mesh_->halfedges(v)) {             //！@！@为点周围的所有向外的半边：对每个半边操作：
			CollapseData cd(mesh_, h);                         //半边h的邻近数据统计
			if (is_collapse_legal(cd)) {                       //半边h是否允许塌缩?可以返回True
				prio = priority(cd);                           //priority()计算这个边的二次误差度量  %%%%%%%
				if (prio != -1.0 && prio < min_prio) {         //找到最小的二次误差=min_prio
					min_prio = prio;
					min_h = h;     
					
				}
			}
		}

		// target found -> put vertex on heap 目标发现→把顶点放在堆上

		if (min_h.is_valid()) {                       //此时的最小代价边min_h可以用
			vpriority_[v] = min_prio;                 //用最小二次误差更新v的队列
			vtarget_[v] = min_h;                      //v的这个邻接半边有最小二次误差
			
			              
			
			if (queue_->is_stored(v))                 //如果v在堆中：即pos_[v] ！= -1为true
				queue_->update(v);                    //更新queue_：更新条目:更改键并更新位置以重新建立堆属性。
			else
				queue_->insert(v);                    //不在堆中就插入v
		}

		// not valid -> remove from heap 无效的→删除从堆
		else {
			if (queue_->is_stored(v))
				queue_->remove(v);

			vpriority_[v] = -1;
			vtarget_[v] = min_h;
			
		}
	}

	//-----------------------------------------------------------------------------

	bool SurfaceMeshQEM::is_collapse_legal(const CollapseData &cd) {
	

		// do not collapse boundary vertices to interior vertices不将边界顶点折叠到内部顶点
		if (mesh_->is_border(cd.v0) && !mesh_->is_border(cd.v1))
			return false;

		// there should be at least 2 incident faces at v0在v0处至少有2个入射面
		if (mesh_->next_around_source(mesh_->next_around_source(cd.v0v1)) ==
			cd.v0v1)
			return false;

		// topological check拓扑检查：is_collapse_ok！！！
		if (!mesh_->is_collapse_ok(cd.v0v1))
			return false;

		//int common_vert = 0;
		//for(auto vv: mesh_->vertices(cd.v0))
		//{
		//	for (auto vv2 : mesh_->vertices(cd.v1))
		//	{
		//		if (vv2 == vv) common_vert++;
		//	}
		//}
		//if (common_vert != 2) return false;
		//vec3 newv = compute_new_v(cd);
		//vpoint_[cd.v1] = newv;
		// remember the positions of the endpoints记住端点的位置p0、p1
		const vec3 p0 = vpoint_[cd.v0];
		const vec3 p1 = vpoint_[cd.v1];

		// check for maximum edge length检查最大边长
		if (edge_length_) {
			for (auto v : mesh_->vertices(cd.v0)) {
				if (v != cd.v1 && v != cd.vl && v != cd.vr) {
					if (norm(vpoint_[v] - p1) > edge_length_)
						return false;
				}
			}
		}

		 //check for flipping normals检查翻转法线
		if (normal_deviation_ == 0.0) {
			vpoint_[cd.v0] = p1;
			for (auto f : mesh_->faces(cd.v0)) {
				if (f != cd.fl && f != cd.fr) {
					vec3 n0 = fnormal_[f];
					vec3 n1 = mesh_->compute_face_normal(f);
					if (dot(n0, n1) < 0.0) {
						vpoint_[cd.v0] = p0;
						return false;
					}
				}
			}
			vpoint_[cd.v0] = p0;
		}

		// check normal cone
		else {
			vpoint_[cd.v0] = p1;

			SurfaceMesh::Face fll, frr;
			if (cd.vl.is_valid())
				fll = mesh_->face(
					mesh_->opposite(mesh_->prev(cd.v0v1)));
			if (cd.vr.is_valid())
				frr = mesh_->face(
					mesh_->opposite(mesh_->next(cd.v1v0)));

			for (auto f : mesh_->faces(cd.v0)) {
				if (f != cd.fl && f != cd.fr) {
					NormalCone nc = normal_cone_[f];
					nc.merge(mesh_->compute_face_normal(f));

					if (f == fll)
						nc.merge(normal_cone_[cd.fl]);
					if (f == frr)
						nc.merge(normal_cone_[cd.fr]);

					if (nc.angle() > 0.5 * normal_deviation_) {
						vpoint_[cd.v0] = p0;
						return false;
					}
				}
			}

			vpoint_[cd.v0] = p0;
		}
		//**********************************************************//
		// check aspect ratio检查纵横比例
		if (aspect_ratio_) {
			float ar0(0), ar1(0);

			for (auto f : mesh_->faces(cd.v0)) {
				if (f != cd.fl && f != cd.fr) {
					// worst aspect ratio after collapse坍塌后的最大纵横比
					vpoint_[cd.v0] = p1;
					ar1 = std::max(ar1, aspect_ratio(f));
					// worst aspect ratio before collapse坍塌前的最大纵横比
					vpoint_[cd.v0] = p0;
					ar0 = std::max(ar0, aspect_ratio(f));
				}
			}

			// aspect ratio is too bad, and it does also not improve纵横比太糟糕了，也没有改善
			if (ar1 > aspect_ratio_ && ar1 > ar0)
				return false;
		}

		return true;
	}

	//-----------------------------------------------------------------------------

	float SurfaceMeshQEM::priority(const CollapseData &cd) {
		// computer quadric error metric
		Quadric Q = vquadric_[cd.v0];
		Q += vquadric_[cd.v1];        //此时Q为边v0v1的二次误差
		//vec3 newv = compute_new_v(cd);
		//return Q(newv);
		return Q(vpoint_[cd.v1]);     //此时为带入v1的能量cost=ei,j
	}

	vec3 SurfaceMeshQEM::compute_new_v(const CollapseData &cd){
		Quadric Q = vquadric_[cd.v0];
		Q += vquadric_[cd.v1];        //此时Q为边v0v1的二次误差Qeij

		//XForm4x4<class Real>* xForm;
		XForm4x4<REAL> xForm;
		XForm4x4<REAL> xForm_inv;
		//XForm4x4<REAL> xForm_test;
		//XForm4x4<REAL> xForm_test_inv;
		vec3 new_v;
		
		xForm.coords[0][0] = Q.a_;  //Q.a_为double，xForm.coords[0][1]为Real类型
		xForm.coords[0][1] = Q.b_;
		xForm.coords[0][2] = Q.c_;
		xForm.coords[0][3] = Q.d_;

		xForm.coords[1][0] = Q.b_;
		xForm.coords[1][1] = Q.e_;
		xForm.coords[1][2] = Q.f_;
		xForm.coords[1][3] = Q.g_;

		xForm.coords[2][0] = Q.c_;
		xForm.coords[2][1] = Q.f_;
		xForm.coords[2][2] = Q.h_;
		xForm.coords[2][3] = Q.i_;

		xForm.coords[3][0] = 0.0;
		xForm.coords[3][1] = 0.0;
		xForm.coords[3][2] = 0.0;
		xForm.coords[3][3] = 1.0;
		
		xForm_inv = xForm.inverse();    //求Qeij的逆矩阵
		new_v[0] = xForm_inv.coords[0][3];
		new_v[1] = xForm_inv.coords[1][3];
		new_v[2] = xForm_inv.coords[2][3];
		if (  isnan(new_v[0]) || isnan(new_v[1]) || isnan(new_v[2]))
			return new_v = (vpoint_[cd.v0] + vpoint_[cd.v1]) / 2;
		//std::cout << "v1:"<<vpoint_[cd.v1] << "\n ";
		return new_v;
	}
	//-----------------------------------------------------------------------------

	void SurfaceMeshQEM::postprocess_collapse(const CollapseData &cd) {
		// update error quadrics 更新v1的二次误差 &&&&  注v1的坐标即为newv
		vquadric_[cd.v1] += vquadric_[cd.v0];
		// update normal cones？？
        if (normal_deviation_) {
            for (auto f : mesh_->faces(cd.v1)) {
                normal_cone_[f].merge(mesh_->compute_face_normal(f));
            }

            if (cd.vl.is_valid()) {
                SurfaceMesh::Face f = mesh_->face(cd.v1vl);
                if (f.is_valid())
                    normal_cone_[f].merge(normal_cone_[cd.fl]);
            }

            if (cd.vr.is_valid()) {
                SurfaceMesh::Face f = mesh_->face(cd.vrv1);
                if (f.is_valid())
                    normal_cone_[f].merge(normal_cone_[cd.fr]);
            }
        }
	}

	//-----------------------------------------------------------------------------
	float SurfaceMeshQEM::aspect_ratio(SurfaceMesh::Face f) const {
		// min height is area/maxLength
		// aspect ratio = length / height
		//              = length * length / area

		SurfaceMesh::VertexAroundFaceCirculator fvit = mesh_->vertices(f);

		const vec3 p0 = vpoint_[*fvit];
		const vec3 p1 = vpoint_[*(++fvit)];
		const vec3 p2 = vpoint_[*(++fvit)];

		const vec3 d0 = p0 - p1;
		const vec3 d1 = p1 - p2;
		const vec3 d2 = p2 - p0;

		const float l0 = length2(d0);
		const float l1 = length2(d1);
		const float l2 = length2(d2);

		// max squared edge length
		const float l = std::max(l0, std::max(l1, l2));

		// triangle area
		float a = norm(cross(d0, d1));

		return l / a;
	}

	//-----------------------------------------------------------------------------

	float SurfaceMeshQEM::distance(SurfaceMesh::Face f, const vec3 &p) const {
		SurfaceMesh::VertexAroundFaceCirculator fvit = mesh_->vertices(f);

		const vec3 p0 = vpoint_[*fvit];
		const vec3 p1 = vpoint_[*(++fvit)];
		const vec3 p2 = vpoint_[*(++fvit)];

		vec3 n;
		return geom::dist_point_triangle(p, p0, p1, p2, n);
	}

	//-----------------------------------------------------------------------------

	SurfaceMeshQEM::CollapseData::CollapseData(SurfaceMesh *sm, SurfaceMesh::Halfedge h) //根据网格和半边作为参数，统合相关数据
		: mesh(sm) {
		v0v1 = h;                         //待塌陷的半边h
		v1v0 = mesh->opposite(v0v1);      //h的对向半边
		v0 = mesh->target(v1v0);          //半边指向的顶点v0，需要移动
		v1 = mesh->target(v0v1);          //半边指向的顶点v1，需要保留
		fl = mesh->face(v0v1);            //h半边对应的面
		fr = mesh->face(v1v0);            //对向半边对应的面

		// get vl
		if (fl.is_valid()) {
			v1vl = mesh->next(v0v1);     //面内的下一个半边v1vL
			vlv0 = mesh->next(v1vl);     //
			vl = mesh->target(v1vl);     //三角面v0v1vL的另一个点vL。
		}

		// get vr
		if (fr.is_valid()) {
			v0vr = mesh->next(v1v0);
			vrv1 = mesh->prev(v0vr);      //返回面内的前半边????!?!?!?!!?
			vr = mesh->source(vrv1);      //返回半边h发出的顶点
		}
	}

	//-----------------------------------------------------------------------------

	void SurfaceMeshQEM::collapse_qem_LML(SurfaceMesh::Halfedge h, vec3 newv)
	{
		//let's make it sure it is actually checked
		assert(is_collapse_ok(h));

		//std::vector<SurfaceMesh::Vertex>  face_points;//将allone_ring内的点集按照邻接顺序排列
		//std::vector<SurfaceMesh::Vertex> allone_ring;//不重复的两个点的邻域点
		//SurfaceMesh::Vertex   pot;


		//SurfaceMesh::Vertex   start_v = mesh_->source(h);          //找到边上的两个点
		//SurfaceMesh::Vertex   end_v = mesh_->target(h);
		//allone_ring.clear();
		//face_points.clear();


		//for (auto vv : this->mesh_->vertices(start_v)) allone_ring.push_back(vv);
		//for (auto bb : this->mesh_->vertices(end_v)) {             //对每个bb
		//	int i = 0;
		//	for (i; i < allone_ring.size(); ++i) {     //在allonering中挨个对比
		//		if (bb == allone_ring[i])                      //找到了一样的bb，
		//			break;                                     //不再找这个bb了
		//	}
		//	if (i == allone_ring.size()) allone_ring.push_back(bb);
		//}

		SurfaceMesh::Halfedge h0 = h;           //需要塌陷
		SurfaceMesh::Halfedge h1 = mesh_->prev(h0);    //返回事件面内的前半边//需要塌陷
		SurfaceMesh::Halfedge o0 = mesh_->opposite(h0);//需要塌陷
		SurfaceMesh::Halfedge o1 = mesh_->next(o0);    //需要塌陷


		//SurfaceMesh::Vertex vh = mesh_->target(h);  //最终保留的顶点，边v0v1中的v1
		//vpoint_[vh] = newv;

		// remove edge
		mesh_->remove_edge(h0);
		
		// remove loops
		if (mesh_->next(mesh_->next(h1)) == h1)
			mesh_->remove_loop(h1);
		if (mesh_->next(mesh_->next(o1)) == o1)
			mesh_->remove_loop(o1);
	}
} // namespace easy3d



