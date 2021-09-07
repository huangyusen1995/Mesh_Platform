

#ifndef ALGO_PLANAR_FEATURE_PRESERVING_H
#define ALGO_PLANAR_FEATURE_PRESERVING_H


#include <easy3d/core/surface_mesh.h>


namespace easy3d {

	
	class Planar_FeaturePreserving {
	public:
	
		static void propagate_planar_component(SurfaceMesh *mesh,
			SurfaceMesh::FaceProperty<int> id,
			SurfaceMesh::Face seed, int cur_id,
			float angle_threshold
		);

		static int enumerate_planar_Featurepreserving(
			SurfaceMesh *mesh, SurfaceMesh::FaceProperty<int> id,                //id为分割标签的数组
			float angle_threshold, float area_threshold, float fitting_threshold);

	};

} 


#endif  // ALGO_PLANAR_FEATURE_PRESERVING_H

