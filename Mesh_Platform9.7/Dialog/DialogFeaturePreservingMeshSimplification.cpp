#include "DialogFeaturePreservingMeshSimplification.h"

#include "..\Algorithm\BilateralNormalFiltering.h"
#include "..\Algorithm\Planar_FeaturePreserving.h"
#include "..\Algorithm\QEM_trian.h"

#include <easy3d/renderer/renderer.h>
#include <easy3d/renderer/setting.h>
#include <easy3d/renderer/camera.h>
#include <easy3d/renderer/clipping_plane.h>
#include <easy3d/renderer/drawable_lines.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/key_frame_interpolator.h>
#include <easy3d/renderer/drawable_triangles.h>
#include <easy3d/renderer/manipulator.h>
#include <easy3d/renderer/transform.h>
#include <easy3d/util/progress.h>

#include "..\Mesh_Platform.h"
#include "..\viewer_mesh_platform.h"


using namespace easy3d;

DialogFeaturePreservingMeshSimplification::DialogFeaturePreservingMeshSimplification(Mesh_PlatformClass *window)
	: QDialog(window) {
	window_ = window;
	viewer_ = window_->viewer();

	const Qt::WindowFlags flags = windowFlags();
	setWindowFlags(flags | Qt::Tool);
	//setWindowFlags(flags | Qt::SubWindow);
	//setWindowFlags(Qt::CustomizeWindowHint | Qt::FramelessWindowHint);

	setupUi(this);

	//layout()->setSizeConstraint(QLayout::SetFixedSize);
	connect(ButtonRunBilateralFilter, SIGNAL(clicked()), this, SLOT(apply_BF()));
	connect(ButtonRunPlanarRegionExtraction, SIGNAL(clicked()), this, SLOT(apply_RegionGrow()));
	connect(ButtonRunQEM, SIGNAL(clicked()), this, SLOT(apply_QEM()));
}


DialogFeaturePreservingMeshSimplification::~DialogFeaturePreservingMeshSimplification() {
}


void DialogFeaturePreservingMeshSimplification::apply_BF()
{
	
	SurfaceMesh *mesh = dynamic_cast<SurfaceMesh *>(viewer_->currentModel());
	if (!mesh)
		return;

	const float theta = BoxBFtheta->value();
	const double Threshold = BoxBFVertexThreshold->value();
	const int Iteration = BoxBFIteration->value();


	MeshFilteringBF bfFilter = MeshFilteringBF(mesh);
	bfFilter.Runfilter(theta,Iteration, Threshold);

	mesh->renderer()->update();
	viewer_->update();
	window_->updateUi();

}

void DialogFeaturePreservingMeshSimplification::apply_RegionGrow()
{
	SurfaceMesh *mesh = dynamic_cast<SurfaceMesh *>(viewer_->currentModel());
	if (!mesh)
		return;


	const float angle = doubleAngle->value();
	const float area_threshold = doubleAreaThreshold->value();
	const float fitting_threshold = doubleFittingThreshold->value();


	const std::string partition_name = "f:planar_partition";
	auto planar_segments = mesh->face_property<int>(partition_name, -1);//auto 
	cur_id=Planar_FeaturePreserving::enumerate_planar_Featurepreserving(mesh, planar_segments, angle, area_threshold, fitting_threshold);
	//SurfaceMeshEnumerator::enumerate_planar_components(mesh, planar_segments, 10.0f);


	const std::string color_name = "f:color_planar_partition";
	auto coloring = mesh->face_property<vec3>(color_name, vec3(0, 0, 0));
	Renderer::color_from_segmentation(mesh, planar_segments, coloring);
	auto faces = mesh->renderer()->get_triangles_drawable("faces");
	faces->set_property_coloring(State::FACE, color_name);


	mesh->renderer()->update();
	viewer_->update();
	window_->updateUi();

}

void DialogFeaturePreservingMeshSimplification::apply_QEM()
{
	
	//SurfaceMesh *mesh = dynamic_cast<SurfaceMesh *>(window_->opened_model);
	SurfaceMesh *mesh = dynamic_cast<SurfaceMesh *>(viewer_->currentModel());
	if (!mesh)
		return;


	const int points_number = BoxPointsNumber->value();
	const float energy_value = BoxEnergyValue->value();
	
	SurfaceMeshQEM simp = SurfaceMeshQEM(mesh);
	simp.initialize(cur_id);
	simp.simplify(points_number);



	mesh->renderer()->update();
	viewer_->update();
	window_->updateUi();

}

