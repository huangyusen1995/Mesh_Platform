#pragma once
#include <easy3d/core/surface_mesh.h>

namespace easy3d
{
	class SurfaceMesh;
}
using namespace easy3d;
class SurfaceMesh_error
{
public:
	SurfaceMesh_error(SurfaceMesh *mesh);
	~SurfaceMesh_error();

	void mean_error(SurfaceMesh *original_mesh);
	void RMS_error(SurfaceMesh *original_mesh);
	void Hausdorff_error(SurfaceMesh *original_mesh);
	void SAMD_mean_error(SurfaceMesh *original_mesh);

private:
	SurfaceMesh *mesh_;
	SurfaceMesh::FaceProperty<vec3>   fnormal;
	double m_b;

};

