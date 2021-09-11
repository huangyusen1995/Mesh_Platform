#pragma once
#include <3rd_party\poisson\Geometry.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/core/heap.h>

#include <set>


namespace easy3d {

	//! A quadric as a symmetrix 4x4 matrix. Used by the error quadric mesh decimation algorithms.作为4x4矩阵对称的二次矩阵。采用误差二次网格塌陷算法
	class Quadric {
	public:
		//! construct quadric from upper triangle of symmetric 4x4 matrix由对称4x4矩阵的上三角形构造二次曲面
		Quadric(double a, double b, double c, double d,
			double e, double f, double g,
			double h, double i,
			double j)
			: a_(a), b_(b), c_(c), d_(d),
			e_(e), f_(f), g_(g),
			h_(h), i_(i),
			j_(j) {}

		//! constructor quadric from given plane equation: ax+by+cz+d=0从给定的平面方程构造二次方程:ax+by+cz+d=0
		Quadric(double a = 0.0, double b = 0.0, double c = 0.0, double d = 0.0)
			: a_(a * a), b_(a * b), c_(a * c), d_(a * d),
			e_(b * b), f_(b * c), g_(b * d),
			h_(c * c), i_(c * d),
			j_(d * d) {}

		//! construct from point and normal specifying a plane指定平面，由点和法线构造
		Quadric(const vec3 &n, const vec3 &p) {
			*this = Quadric(n[0], n[1], n[2], -dot(n, p));
		}

		//! set all matrix entries to zero将所有矩阵项设为零
		void clear() { a_ = b_ = c_ = d_ = e_ = f_ = g_ = h_ = i_ = j_ = 0.0; }

		//! add given quadric to this quadric；把给定的二次曲线q加到这个二次曲线上
		Quadric &operator+=(const Quadric &q) {
			a_ += q.a_;
			b_ += q.b_;
			c_ += q.c_;
			d_ += q.d_;
			e_ += q.e_;
			f_ += q.f_;
			g_ += q.g_;
			h_ += q.h_;
			i_ += q.i_;
			j_ += q.j_;
			return *this;
		}

		//! multiply quadric by a scalar二次方程乘以一个标量
		Quadric &operator*=(double s) {
			a_ *= s;
			b_ *= s;
			c_ *= s;
			d_ *= s;
			e_ *= s;
			f_ *= s;
			g_ *= s;
			h_ *= s;
			i_ *= s;
			j_ *= s;
			return *this;
		}

		//! evaluate quadric Q at position p by computing (p^T * Q * p)；通过计算(p^T * Q * p)求二次Q在p位置的值，用实例Quadric_(p)来调用
		double operator()(const vec3 &p) const {
			const double x(p[0]), y(p[1]), z(p[2]);
			return a_ * x * x + 2.0 * b_ * x * y + 2.0 * c_ * x * z + 2.0 * d_ * x
				+ e_ * y * y + 2.0 * f_ * y * z + 2.0 * g_ * y
				+ h_ * z * z + 2.0 * i_ * z
				+ j_;
		}

	public:

		double a_, b_, c_, d_,
			e_, f_, g_,
			h_, i_,
			j_;
	};

	//=============================================================================
		/// A class implementing a normal cone.一个实现法锥的类。
	class NormalCone {
	public:
		//! default constructor (not initialized)
		NormalCone() {}

		//! Initialize cone with center (unit vector) and angle (radius in radians)用圆心(单位矢量)和角度(半径为弧度)初始化圆锥
		NormalCone(const vec3 &normal, float angle = 0.0)
			: center_normal_(normal), angle_(angle) {
		}

		//! returns center normal
		const vec3 &center_normal() const { return center_normal_; }

		//! returns size of cone (radius in radians)
		float angle() const { return angle_; }

		//! merge *this with n.
		NormalCone &merge(const vec3 &n) { return merge(NormalCone(n)); }

		//! merge *this with nc. *this will then enclose both cones.
		NormalCone &merge(const NormalCone &nc) {
			const float dp = dot(center_normal_, nc.center_normal_);

			// axes point in same direction坐标轴指向同一方向
			if (dp > 0.99999) {
				angle_ = std::max(angle_, nc.angle_);
			}

			// axes point in opposite directions
			else if (dp < -0.99999) {
				angle_ = 2 * M_PI;
			}
			else {
				// new angle
				float center_angle = acos(dp);
				float min_angle = std::min(-angle_, center_angle - nc.angle_);
				float max_angle = std::max(angle_, center_angle + nc.angle_);
				angle_ = 0.5 * (max_angle - min_angle);

				// axis by SLERP
				float axis_angle = 0.5 * (min_angle + max_angle);
				center_normal_ = ((center_normal_ * sin(center_angle - axis_angle) +
					nc.center_normal_ * sin(axis_angle)) /
					sin(center_angle));
			}

			return *this;
		}

	private:
		vec3 center_normal_;
		float angle_;
	};

	/**
	 * \brief Surface mesh simplification based on approximation error and fairness criteria.基于近似误差和公平性准则的表面网格简化
	 * \class SurfaceMeshSimplification easy3d/algo/surface_mesh_simplification.h
	 * \details It performs incremental greedy mesh simplification based on halfedge collapses. See the following paper
	 * for more details:
	 *  - Michael Garland and Paul Seagrave Heckbert. Surface simplification using quadric error metrics. SIGGRAPH 1997.
	 *  - Leif Kobbelt et al. A general framework for mesh decimation. In Proceedings of Graphics Interface, 1998.
	 */
	class SurfaceMeshQEM {
	public:
		//! Construct with mesh to be simplified.
		SurfaceMeshQEM(SurfaceMesh *mesh);

		// destructor
		~SurfaceMeshQEM();

		//! Initialize with given parameters.
		void initialize(int cur_id=0, float aspect_ratio = 0.0, float edge_length=0.0, float normal_deviation = 0.0 );

		//! Simplify mesh to \p n vertices.
		void simplify(unsigned int n_vertices);

	private:
		//! Store data for an halfedge collapse
		/*
					vl
					*
				   / \
				  /   \
				 / fl  \
			 v0 *------>* v1
				 \ fr  /
				  \   /
				   \ /
					*
					vr
		*/
		struct CollapseData {
			CollapseData(SurfaceMesh *sm, SurfaceMesh::Halfedge h);    //根据网格和半边作为参数，统合相关数据

			SurfaceMesh *mesh;

			SurfaceMesh::Halfedge v0v1; // Halfedge to be collapsed
			SurfaceMesh::Halfedge v1v0; // Reverse halfedge
			SurfaceMesh::Vertex v0;     // Vertex to be removed
			SurfaceMesh::Vertex v1;     // Remaining vertex
			SurfaceMesh::Face fl;       // Left face
			SurfaceMesh::Face fr;       // Right face
			SurfaceMesh::Vertex vl;     // Left vertex
			SurfaceMesh::Vertex vr;     // Right vertex
			SurfaceMesh::Halfedge v1vl, vlv0, v0vr, vrv1;
		};

		//! Heap interface 堆接口
		class HeapInterface {
		public:
			HeapInterface(SurfaceMesh::VertexProperty<float> prio, SurfaceMesh::VertexProperty<int> pos)
				: prio_(prio), pos_(pos) {
			}

			bool less(SurfaceMesh::Vertex v0, SurfaceMesh::Vertex v1) { return prio_[v0] < prio_[v1]; }

			bool greater(SurfaceMesh::Vertex v0, SurfaceMesh::Vertex v1) { return prio_[v0] > prio_[v1]; }

			int get_heap_position(SurfaceMesh::Vertex v) { return pos_[v]; }

			void set_heap_position(SurfaceMesh::Vertex v, int pos) { pos_[v] = pos; }//将v点的堆位置重置为pos)

		private:
			SurfaceMesh::VertexProperty<float> prio_;
			SurfaceMesh::VertexProperty<int> pos_;
		};

		typedef Heap<SurfaceMesh::Vertex, HeapInterface> PriorityQueue;

		typedef std::vector<vec3> Points;

	private:
		// put the vertex v in the priority queue将顶点v放入优先队列
		void enqueue_vertex(SurfaceMesh::Vertex v);

		// is collapsing the halfedge h allowed?半边h是否允许塌缩?
		bool is_collapse_legal(const CollapseData &cd);

		// what is the priority of collapsing the halfedge h;塌陷半边h的优先级是什么
		float priority(const CollapseData &cd);

		//计算新的点！！！！！！
		vec3 compute_new_v(const CollapseData &cd);

		// postprocess halfedge collapse后处理半边塌陷
		void postprocess_collapse(const CollapseData &cd);

		// compute aspect ratio for face f计算面f的纵横比
		float aspect_ratio(SurfaceMesh::Face f) const;
		// compute distance from p to triagle f计算从p到三角f的距离
		float distance(SurfaceMesh::Face f, const vec3 &p) const;

	public:
		void collapse_qem_LML(SurfaceMesh::Halfedge h, vec3 newv);
		//void adjust_outgoing_halfedge(SurfaceMesh::Vertex v);//如果v是一个边界顶点，则确保顶点v的外向半边是一个边界半边
		//void remove_edge(SurfaceMesh::Halfedge h);
		//void remove_loop(SurfaceMesh::Halfedge h);
	private:
		SurfaceMesh *mesh_;
		/////
		//SurfaceMesh::VertexProperty<bool>  vdeleted_;
		//SurfaceMesh::EdgeProperty<bool>    edeleted_;
		//SurfaceMesh::FaceProperty<bool>    fdeleted_;
		//unsigned int deleted_vertices_;
		//unsigned int deleted_edges_;
		//unsigned int deleted_faces_;
		//bool garbage_;
		/////
		bool initialized_;

		SurfaceMesh::VertexProperty<float> vpriority_;              //顶点属性的小数数组，优先级
		SurfaceMesh::VertexProperty<SurfaceMesh::Halfedge> vtarget_;//顶点属性的半边数组，顶点的出边

		//SurfaceMesh::VertexProperty<vec3> new_vertices;

		SurfaceMesh::VertexProperty<int> heap_pos_;                 //顶点属性的整数数组，堆
		SurfaceMesh::VertexProperty<Quadric> vquadric_;       //顶点属性的二次误差数组
		
		SurfaceMesh::FaceProperty<Points> face_points_;       //面属性的点数组，面的邻点

		SurfaceMesh::VertexProperty<vec3> vpoint_;            //顶点属性的坐标数组，为点的坐标
		SurfaceMesh::FaceProperty<vec3> fnormal_;             //面属性的坐标数组，为面的法向量

		//SurfaceMesh::VertexProperty<bool> vfeature_;          //顶点属性的布尔数组，
		//SurfaceMesh::EdgeProperty<bool> efeature_;            //边属性的布尔数组，

		SurfaceMesh::FaceProperty<int> planar_segments;

		PriorityQueue *queue_;

		float normal_deviation_;
		float edge_length_;
		float aspect_ratio_;
		SurfaceMesh::FaceProperty<NormalCone> normal_cone_;   //面属性的法线锥数组
	

	};

} // namespace easy3d