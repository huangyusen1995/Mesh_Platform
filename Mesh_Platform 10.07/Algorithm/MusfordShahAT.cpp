#include "MusfordShahAT.h"

#include <easy3d/algo/surface_mesh_geometry.h>
#include ".\3rd_party\eigen\Eigen\Sparse"
#include ".\3rd_party\eigen\Eigen\Dense"


using namespace easy3d;
using namespace std;

using SparseMatrix = Eigen::SparseMatrix<double>;
using Triplet = Eigen::Triplet<double>;


MumfordShah_AT::MumfordShah_AT(SurfaceMesh *mesh) : mesh_(mesh)
{
	m_triangle_num = mesh_->n_faces();
	m_edge_num = mesh_->n_edges();
	m_triangle_edge_sgns = vector<vector<int>>(m_triangle_num, vector<int>(3, 0));
	m_triangle_edge_id = vector<vector<int>>(m_triangle_num, vector<int>(3, 0));
	m_edges_legth = vector<double>(m_edge_num, 0);
	m_edge_baryEdges = vector<vector<double>>(m_edge_num, vector<double>(4, 0));
	m_edge_operator_indices = vector<vector<int>>(m_edge_num, vector<int>(5, 0));
	mesh_->update_face_normals();
	fnormal_ = mesh_->get_face_property<vec3>("f:normal");
	normal_filtered = mesh_->get_face_property<vec3>("f:normal");
	m_gradients = vector<vec3>(m_edge_num, { 0, 0, 0 });



}
MumfordShah_AT::~MumfordShah_AT()
{
	//mesh_->remove_edge_property(V);
}


void MumfordShah_AT::setParams(double alpha,
	double beta, double threshold,
	double epsilon,
	int normItrNum, int verItrNum)
{
	m_alpha = alpha;
	m_beta = beta;
	m_threshold = threshold;
	m_epsilon = epsilon;
	m_normal_itr_num = normItrNum;
	m_vertex_itr_num = verItrNum;

	std::cout << "\nalpha :" << m_alpha << std::endl;
	std::cout << "beta :" << m_beta << std::endl;
	std::cout << "epsilon :" << m_epsilon << std::endl;
	std::cout << "m_triangle_num :" << m_triangle_num << std::endl;
}

vector<int> MumfordShah_AT::FindVertexConnectEdges(SurfaceMesh::Vertex one_v)
{
	vector<int> fs;
	for (auto h : mesh_->halfedges(one_v))
	{
		SurfaceMesh::Edge one_e = mesh_->edge(h);
		fs.push_back(one_e.idx());
	}
	return fs;
}

void MumfordShah_AT::calculate_operators_indices()
{

	///////////////////////////////����Ϊ�洢�����αߵ�id���洢�����ε���������///////////////////////////
	vector<int>		 sgn = { 0,0,0 };
	vector<int>		 a_triangle_edge_id = { 0,0,0 };
	SurfaceMesh::Edge one_e;

	for (auto f : mesh_->faces())
	{
		//sgn.clear();
		//cout << "�棺" << f << endl;
		unsigned int hi = 0;
		for (auto h : mesh_->halfedges(f))
		{
			one_e = mesh_->edge(h);
			a_triangle_edge_id[hi] = one_e.idx();
			sgn[hi] = -(int)pow(-1.0, h.idx());
			//cout << "sgn" << hi <<"="<< sgn[hi] << endl;
			//cout << "h" << h.idx() << endl;
			hi++;
		}
		m_triangle_edge_id[f.idx()] = a_triangle_edge_id;
		m_triangle_edge_sgns[f.idx()] = sgn;
			/*cout << "m_triangle_edge_id" << f.idx() <<"=" << m_triangle_edge_id[f.idx()][0] <<" ";
			cout << m_triangle_edge_id[f.idx()][1] << " ";
			cout << m_triangle_edge_id[f.idx()][2] << endl;
			cout << "m_triangle_edge_sgns" << f.idx() <<"=" << m_triangle_edge_sgns[f.idx()][0] <<" ";
			cout << m_triangle_edge_sgns[f.idx()][1] << " ";
			cout << m_triangle_edge_sgns[f.idx()][2] << endl;*/
	}
	///////////////////////////////����Ϊ�洢�����αߵ�id���洢�����ε���������////////////////////////////

	///////////////////////////////����Ϊ�洢�������ߵĳ��Ⱥ�id//////////////////////////////////

	vector<int> opr(5, 0);
	vector<double> edge_baryEdges(4, 0);
	vector<int> face_edges1, face_edges2;
	vector<int> vertex_edges;
	vector<int>::iterator s;
	for (auto e : mesh_->edges())
	{
		m_edges_legth[e.idx()] = mesh_->edge_length(e);
		//��e������������
		SurfaceMesh::Face f1 = mesh_->face(e, 0);
		SurfaceMesh::Face f2 = mesh_->face(e, 1);

		//��¼�����ߵ�id
		opr[4] = e.idx();
		/*cout << "s4:" << opr[4] << endl;*/
		//��������߲����ڱ�Ե
		if (f1.is_valid() && f2.is_valid())
		{
			//�õ�����������а��
			face_edges1 = m_triangle_edge_id[f1.idx()];
			face_edges2 = m_triangle_edge_id[f2.idx()];
			/*cout << "face_edges1 " << f1.idx() << " = " << face_edges1[0] << " ";
			cout << face_edges1[1] << " " << face_edges1[2] << endl;
			cout << "face_edges2 " << f2.idx() << " = " << face_edges2[0] << " ";
			cout << face_edges2[1] << " " << face_edges2[2] << endl;*/
			////ѭ��e�н�С�İ����ŵ�id
			////�ҵ�����������ӱ�
			vertex_edges = FindVertexConnectEdges(mesh_->vertex(e, 0));
			for (size_t k = 0; k < vertex_edges.size(); ++k)
			{
				s = find(face_edges1.begin(), face_edges1.end(), vertex_edges[k]);
				//cout << "face_edges1.begin():"<<face_edges1[0]<<endl;
				//cout << "face_edges1.end():" << face_edges1[2] << endl;
				if (s != face_edges1.end())
				{
					if (vertex_edges[k] != e.idx())
						opr[0] = vertex_edges[k];
					//cout<<"s0:"<< opr[0]<<endl;
				}
				s = find(face_edges2.begin(), face_edges2.end(), vertex_edges[k]);
				if (s != face_edges2.end())
				{
					if (vertex_edges[k] != e.idx())
						opr[1] = vertex_edges[k];
					//cout << "s1:" << opr[1] << endl;
				}
			}

			//ѭ��e�нϴ�İ����ŵ�id
			//�ҵ�����������ӱ�
			vertex_edges = FindVertexConnectEdges(mesh_->vertex(e, 1));
			for (size_t k = 0; k < vertex_edges.size(); ++k)
			{
				s = find(face_edges1.begin(), face_edges1.end(), vertex_edges[k]);
				//cout << "face_edges1.begin():"<<face_edges1[0]<<endl;
				//cout << "face_edges1.end():" << face_edges1[2] << endl;
				if (s != face_edges1.end())
				{
					if (vertex_edges[k] != e.idx())
						opr[2] = vertex_edges[k];
					//cout<<"s2:"<< opr[2]<<endl;
				}
				s = find(face_edges2.begin(), face_edges2.end(), vertex_edges[k]);
				if (s != face_edges2.end())
				{
					if (vertex_edges[k] != e.idx())
						opr[3] = vertex_edges[k];
					//cout << "s3:" << opr[3] << endl;
				}
			}

			vec3 f1_center = geom::centroid(mesh_, f1);
			vec3 f2_center = geom::centroid(mesh_, f2);
			vec3 v_e0 = mesh_->position(mesh_->vertex(e, 0));
			vec3 v_e1 = mesh_->position(mesh_->vertex(e, 1));
			edge_baryEdges[0] = norm(f1_center - v_e0);
			edge_baryEdges[1] = norm(f2_center - v_e0);
			edge_baryEdges[2] = norm(f1_center - v_e1);
			edge_baryEdges[3] = norm(f2_center - v_e1);
			//cout << "m_vertices_in" << (mesh_->vertex(e, 0)).idx() << " : " << v_e0 << endl;
			//cout << "m_tri_baryCentor" << f1.idx() << " : " << f1_center << endl;
		}
		m_edge_operator_indices[e.idx()] = opr;
		m_edge_baryEdges[e.idx()] = edge_baryEdges;
		//cout << "m_edge_baryEdges " << e.idx() << " = " << edge_baryEdges[0] << " ";
		//cout << edge_baryEdges[1] << " " << edge_baryEdges[2] << " " << edge_baryEdges[3] << endl;
		//cout << "m_edge_operator_indices " << e.idx() << " = " << m_edge_operator_indices[e.idx()][0] << " ";
		//cout << m_edge_operator_indices[e.idx()][1] << " " << m_edge_operator_indices[e.idx()][2] << " " << m_edge_operator_indices[e.idx()][3] << " ";
		//cout << m_edge_operator_indices[e.idx()][4] << endl;

	}

	///////////////////////////////����Ϊ�洢�������ߵĳ��Ⱥ�id//////////////////////////////////

}


//���N����
void MumfordShah_AT::solveNSubProblem()
{

///////////////////////////////����N���������/////////////////////////////////////
	SparseMatrix matrixA_(m_triangle_num, m_triangle_num);
	Eigen::MatrixXd B(m_triangle_num, 3);
	vector< Eigen::Triplet<double> > coeff_triple; coeff_triple.clear();

	for (auto f : mesh_->faces())
	{
		//�Խ������Ϊ��Ӧ������id��	alpha*Si
		double triangle_area = geom::triangle_area(mesh_, f);
		coeff_triple.push_back(Eigen::Triplet<double>(f.idx(), f.idx(), m_alpha*triangle_area));
		//���ڴ������ε������ڽ������Σ���������ڣ���ô�ڶԽ����ϼ���	(�ڽӱߵ�Vֵ��ƽ��)*�����ڽӱߵĳ���
		//�����ж�Ӧ���ڽ����Ӧid�����ϣ�����	-(�ڽӱߵ�Vֵ��ƽ��)*�����ڽӱߵĳ���		
		for (auto h : mesh_->halfedges(f))
		{
			SurfaceMesh::Face f_f = mesh_->face(mesh_->opposite(h));
			if (mesh_->is_valid(f_f))
			{
				SurfaceMesh::Edge f_f_e = mesh_->edge(h);
				double f_f_e_length = mesh_->edge_length(f_f_e);
				//cout << f_f_e_length <<endl;
				coeff_triple.push_back(Eigen::Triplet<double>(f.idx(), f.idx(),pow(V[f_f_e],2)* f_f_e_length));
				coeff_triple.push_back(Eigen::Triplet<double>(f.idx(), f_f.idx(),-pow(V[f_f_e], 2)*f_f_e_length));
			}
		}
	}
	
	matrixA_.setFromTriplets(coeff_triple.begin(), coeff_triple.end());
	matrixA_.makeCompressed();
	Eigen::SimplicialLDLT<SparseMatrix> solver_;
	solver_.compute(matrixA_);
	//cout << "matrixA_" << matrixA_ << endl;
    
////////////////////////////////���N����/////////////////////////////////////////

	Eigen::MatrixXd vecB(m_triangle_num, 3);
	Eigen::MatrixX3d filtered_normals_matrix;
	vec3  temp = {0,0,0};
	for (auto f : mesh_->faces())
	{
		double triangle_area = geom::triangle_area(mesh_, f);
		temp = m_alpha * triangle_area * fnormal_[f];
		//vecB�Ƿ������˻���x��y��z
		vecB(f.idx(), 0) = temp[0];
		vecB(f.idx(), 1) = temp[1];
		vecB(f.idx(), 2) = temp[2];
	}
	//���N����
	filtered_normals_matrix = solver_.solve(vecB);
	filtered_normals_matrix.rowwise().normalize();	
	
	for (auto f : mesh_->faces())
	{
		N[f][0] = filtered_normals_matrix(f.idx(),0);
		N[f][1] = filtered_normals_matrix(f.idx(), 1);
		N[f][2] = filtered_normals_matrix(f.idx(), 2);
	}
}


//���V����
void MumfordShah_AT::solveVSubProblem()
{
///////////////////////////////����V���������/////////////////////////////////////
	//�����Ǳ�����*������
	SparseMatrix matrixVA_(m_edge_num, m_edge_num);
	matrixVA_.reserve(Eigen::VectorXi::Constant(m_edge_num, 5));
	vector< Eigen::Triplet<double> > coeff_triple; coeff_triple.clear();
	
	//�ݶ�
	vec3 grad = {0,0,0};
	for (auto e : mesh_->edges())
	{
		//���e�Ǳ�Ե�ߣ���ô�ݶ�Ϊ0������Ϊ�淨����	N[������]-N[�Լ���]
		if (mesh_->is_border(e))
		{
			grad = { 0, 0, 0 };
		}
		else
		{
			SurfaceMesh::Face f_m = mesh_->face(e, 0);
			SurfaceMesh::Face f_opposite = mesh_->face(e, 1);
			grad = N[f_opposite] - N[f_m];
		}
		m_gradients[e.idx()] = grad;
		//�߾���ĶԽ����϶�����	(2*(�����´��ݶȣ��ߵ����淨�������)+(beta/(2*epsilon)))*�˱ߵĳ���
		coeff_triple.push_back(Eigen::Triplet<double>(e.idx(), e.idx(), (2 * length2(grad) + (m_beta / (2 * m_epsilon)))*m_edges_legth[e.idx()]));
		for (size_t j = 0; j < m_edge_operator_indices[e.idx()].size() - 1; ++j)
		{
			//�߾���ĶԽ����϶�����	-2*beta*epsilon*�����ڽӱ߼е��߶�֮һ�ĳ���
			coeff_triple.push_back(Eigen::Triplet<double>(e.idx(), e.idx(), 2 * m_beta*m_epsilon*m_edge_baryEdges[e.idx()][j]));
			//�߾����i�еĶ�Ӧj�Խ�����	2*beta*epsilon*�����ڽӱ߼е��߶�֮һ�ĳ���
			coeff_triple.push_back(Eigen::Triplet<double>(e.idx(), m_edge_operator_indices[e.idx()][j], -2 * m_beta*m_epsilon*m_edge_baryEdges[e.idx()][j]));

		}

	}

	matrixVA_.setFromTriplets(coeff_triple.begin(), coeff_triple.end());
	matrixVA_.makeCompressed();
	Eigen::SimplicialLDLT<SparseMatrix>  m_solver;
	m_solver.compute(matrixVA_);

////////////////////////////////���V����/////////////////////////////////
	Eigen::MatrixXd vecB(m_edge_num, 1);
	Eigen::MatrixXd x(m_edge_num, 1);
	for(auto e:mesh_->edges())
	{
		//���ڱ������ÿ������i	(beta/(2*epsilon))*�˱߳���
		vecB(e.idx()) = (m_beta / (2 * m_epsilon))*m_edges_legth[e.idx()];
	}

	x = m_solver.solve(vecB);
	for (auto e:mesh_->edges())
	{
		V[e] = x(e.idx());
		//cout << "V" << e.idx() << " : " << V[e] << endl;
	}

}

double MumfordShah_AT::stopItr()
{
	double normal_diff = 0;
	//�淨�����Ĳ�ֺ�Ϊ	ÿ�������������*(������(���淨����-��һ�˼���Ĵ��淨����))
	for (auto f : mesh_->faces())
	{
		double f_area = geom::triangle_area(mesh_, f);
		normal_diff += f_area * norm(N[f] - previousN[f]);
	}
	return normal_diff;
}

double MumfordShah_AT::VstopItr()
{
	double normal_diff = 0;
	//�淨�����Ĳ�ֺ�Ϊ	����ֵ�µ�ÿ���ߵĳ���*(������(�˱ߵ�Vֵ-��һ�˼���Ĵ˱ߵ�Vֵ))
	for (auto e : mesh_->edges())
	{
		normal_diff += abs(V[e] - previousV[e])*m_edges_legth[e.idx()];

	}
	return normal_diff;
}

double MumfordShah_AT::Energy()
{
	double s = 0, ReEngry = 0, FidelityEnergy = 0, ATEngry = 0, energy = 0, vgradient = 0;

	//��Ϊ	(1/2)*alpha*�淨�������
	FidelityEnergy = (1.0 / 2) * m_alpha * stopItr();

	vector<double> v_edge= vector<double>(m_edge_num, 0);
	for (auto e : mesh_->edges())
	{
		v_edge[e.idx()] = V[e];
	}


	//���ڱ�����
	for (size_t i = 0; i < m_edge_num; ++i)
	{
		//RE����	(�˱���Vֵ��ƽ��)*�����´˱��ݶ�(�ڽ����������)*�߳���
		ReEngry += pow(v_edge[i], 2)*length2(m_gradients[i])*m_edges_legth[i];
		//AT����	(beta/(4*epsilon))*((1-�˱��ϵ�Vֵ)��ƽ��)*�߳���
		ATEngry += (m_beta / (4 * m_epsilon))*pow((1 - v_edge[i]), 2)*m_edges_legth[i];
		for (size_t j = 0; j < m_edge_operator_indices[i].size() - 1; ++j)
		{
			//vg����	�����ڽӱߵ�Vֵ-����vֵ
			vgradient = v_edge[m_edge_operator_indices[i][j]] - v_edge[m_edge_operator_indices[i][4]];
			//AT�ټ���	vg��ƽ��*beta*epsilon*�����ڽӱ��뱾��֮����ߵĳ���
			ATEngry += pow(vgradient, 2)*m_beta*m_epsilon*m_edge_baryEdges[i][j];
		}
	}

	//������
	energy = ReEngry + FidelityEnergy + ATEngry;
	return energy;

}

//�������
void MumfordShah_AT::MSAT_vertexUpdating()
{
	vector<SurfaceMesh::Vertex>  tvs;
	vec3  new_vertex;
	vec3 vertex_displacement = vec3(0, 0, 0);
	SurfaceMesh::VertexProperty<vec3> vertices_in = mesh_->get_vertex_property<vec3>("v:point");

	vec3 v0, v1, v2, tc, fnormal, pij;
	for (int k = 0; k < m_vertex_itr_num; ++k)
	{
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
			vertices_in[v] += vertex_displacement;
		}
	}

	std::cout << "vertex Itrer Numbers:" << m_vertex_itr_num << "times.\n";
}

//

void MumfordShah_AT::MSAT_normalFiltering()
{
	previousN = mesh_->face_property<vec3>("MSAT:previousN", vec3(0, 0, 0));
	N = mesh_->face_property<vec3>("MSAT:N", vec3(0, 0, 0));
    V = mesh_->edge_property<double>("MSAT:V",1.0);
	previousV = mesh_->edge_property<double>("MSAT:previousV", 1.0);

	double  d = 0.0, eps = 1e-5;
	double energy = 0.0;
	int itr = 0;
	//vector<double> itr_idx, energy_idx;

	calculate_operators_indices();
	do
	{
		//solve N sub-problem
		solveNSubProblem();
		
		//solve v sub-problem
		solveVSubProblem();
		
		//energy = Energy();
		//itr_idx.push_back(itr);
		//energy_idx.push_back(energy);

		d = VstopItr();
		previousV = V;
		previousN = N;
	} while (d >= eps && ++itr < m_normal_itr_num);
	normal_filtered = N;

	//save v to files
	std::cout << "start save v...\n";
	ofstream outfile;
	outfile.open("E:/workspace/histogram/V_start.txt", ios::binary | ios::in | ios::out);
	for (auto e:mesh_->edges())
	{
		outfile << V[e] << "\n";
	}
	outfile.close();

	calFeatureEdge();




}

void MumfordShah_AT::filtering()
{
	time_t start, end, wstart, wend, vstart, vend;
	start = clock();
	wstart = start;
	// 1. normal filtering
	std::cout << "\nentering mumford shah AT alg...\n";
	MSAT_normalFiltering();

	end = clock();
	cout << "Normal Filtering time consuming:" << (end - start) / (double)(CLOCKS_PER_SEC) << "s.\n";

	// 2. vertex updating
	vstart = clock();
	cout << "Start Sun Vertex Updating..." << endl;
	MSAT_vertexUpdating();
	
	vend = clock();
	cout << "Vertex updating time consuming:" << (vend - vstart) / (double)(CLOCKS_PER_SEC) << "s.\n";

	end = clock();
	wend = end;
	cout << "Sum time consuming:" << (wend - wstart) / (double)(CLOCKS_PER_SEC) << " s.\n";




}

void MumfordShah_AT::calFeatureEdge()
{
	e_feature = mesh_->edge_property("e:feature", false);
	v_feature = mesh_->vertex_property("v:feature", false);

    for (auto v : mesh_->vertices())
		v_feature[v] = false;

    for (auto e : mesh_->edges())
		e_feature[e] = false;
	//vector<vec3> m_feature_Edges;
	m_feature_Vertices = vector<vec3>(2*m_edge_num, { 0, 0, 0 });
	//vec3 a_feature_Edges;
	vec3 a_feature_Vertices0;
	vec3 a_feature_Vertices1;

	SurfaceMesh::Vertex edge_vertex, edge_vertex1;
	
	for (auto e : mesh_->edges())
	{
		if (V[e] <= m_threshold)
		{

			e_feature[e] = true;
			// �ɱ�������������
			edge_vertex = mesh_->vertex(e, 0);
			edge_vertex1 = mesh_->vertex(e, 1);

			v_feature[edge_vertex] = true;
			v_feature[edge_vertex1] = true;
			//a_feature_Edges = V[e];
			a_feature_Vertices0 = mesh_->position(edge_vertex);
			a_feature_Vertices1 = mesh_->position(edge_vertex1);
		}
		else
		{
			a_feature_Vertices0 = { 0,0,0 };
			a_feature_Vertices1 = { 0,0,0 };
		}

		m_feature_Vertices.push_back(a_feature_Vertices0);
		m_feature_Vertices.push_back(a_feature_Vertices1);
		m_feature_edges_indices.push_back(edge_vertex.idx());
		m_feature_edges_indices.push_back(edge_vertex1.idx());

			//cout << "V" << e.idx() << " : " << V[e] << endl;
			
		
	}

	


}

