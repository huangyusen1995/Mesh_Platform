#include "DialogFeaturePreservingMeshSimplification.h"

#include "..\Algorithm\BilateralNormalFiltering.h"
#include "..\Algorithm\Planar_FeaturePreserving.h"
#include "..\Algorithm\QEM_trian.h"

#include <easy3d/renderer/renderer.h>
#include <easy3d/renderer/drawable_lines.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/drawable_triangles.h>
#include <easy3d/util/file_system.h>

#include "..\Mesh_Platform.h"
#include "..\viewer_mesh_platform.h"
#include "..\Algorithm\SurfaceMesh_error.h"

using namespace easy3d;

DialogFeaturePreservingMeshSimplification::DialogFeaturePreservingMeshSimplification(Mesh_PlatformClass *window)
	: QDialog(window) {
	window_ = window;
	viewer_ = window_->viewer();

	const Qt::WindowFlags flags = windowFlags();
	setWindowFlags(flags | Qt::Tool);

	cur_id = 0;

	copyorigin = NULL;
	copysecond = NULL;
	copythird = NULL;

	setupUi(this);

	connect(ButtonRunBilateralFilter, SIGNAL(clicked()), this, SLOT(apply_BF()));
	connect(ButtonRunPlanarRegionExtraction, SIGNAL(clicked()), this, SLOT(apply_RegionGrow()));
	connect(ButtonRunQEM, SIGNAL(clicked()), this, SLOT(apply_QEM()));
	connect(checkBox_FacesColoring, SIGNAL(toggled(bool)), this, SLOT(setFacesColoring()));
	connect(checkBox_traditionalQEM, SIGNAL(toggled(bool)), this, SLOT(setQEMversion()));
}


DialogFeaturePreservingMeshSimplification::~DialogFeaturePreservingMeshSimplification() {
}


void DialogFeaturePreservingMeshSimplification::apply_BF()
{
	SurfaceMesh *model = dynamic_cast<SurfaceMesh *>(viewer_->originModel());
	if (!model)
	{
		std::cout << "Please load the original model first" << std::endl;
		return;
	}

	if (copyorigin != NULL) {
		viewer_->deleteModel(copyorigin);
		copyorigin = NULL;
		if (copysecond != NULL)
		{
			viewer_->deleteModel(copysecond);
			copysecond = NULL;
			if (copythird != NULL)
			{
				viewer_->deleteModel(copythird);
				copythird = NULL;
			}
		}
	}
	copyorigin = new SurfaceMesh(*dynamic_cast<SurfaceMesh *>(model));

	if (copyorigin) {
		const std::string &name = file_system::name_less_extension(model->name()) + "_BF";
		copyorigin->set_name(name);
		viewer_->addModel(copyorigin);
		viewer_->setCurrentModel(copyorigin);

		SurfaceMesh *mesh = dynamic_cast<SurfaceMesh *>(viewer_->currentModel());
		viewer_->setShowOnly();

		if (!mesh)
			return;

		const float theta = BoxBFtheta->value();
		const double Threshold = BoxBFVertexThreshold->value();
		const int Iteration = BoxBFIteration->value();

		MeshFilteringBF bfFilter = MeshFilteringBF(mesh);
		bfFilter.Runfilter(theta, Iteration, Threshold);

		mesh->renderer()->update();
		viewer_->update();
		window_->updateUi();
	}
}

void DialogFeaturePreservingMeshSimplification::apply_RegionGrow()
{
	if ((viewer_->secondModel()) == nullptr)
	{
		SurfaceMesh *model_one = dynamic_cast<SurfaceMesh *>(viewer_->originModel());
		copyorigin = new SurfaceMesh(*dynamic_cast<SurfaceMesh *>(model_one));
		if (copyorigin) {
			copyorigin->set_name("originModelcopy");
			viewer_->addModel(copyorigin);
			viewer_->setCurrentModel(copyorigin);
			viewer_->update();
		}
	}

	SurfaceMesh *model = dynamic_cast<SurfaceMesh *>(viewer_->secondModel());
	
	if (copysecond != NULL)
	{
		viewer_->deleteModel(copysecond);
		copysecond = NULL;
		if (copythird != NULL)
		{
			viewer_->deleteModel(copythird);
			copythird = NULL;
		}
	}
	copysecond = new SurfaceMesh(*dynamic_cast<SurfaceMesh *>(model));

	if (copysecond) {
		const std::string &name = file_system::name_less_extension(model->name()) + "_RG";
		copysecond->set_name(name);
		viewer_->addModel(copysecond);
		viewer_->setCurrentModel(copysecond);

		SurfaceMesh *mesh = dynamic_cast<SurfaceMesh *>(viewer_->currentModel());

		viewer_->setShowOnly();

		if (!mesh)
			return;

		const float angle = doubleAngle->value();
		const float area_threshold = doubleAreaThreshold->value();
		const float fitting_threshold = doubleFittingThreshold->value();

		const std::string partition_name = "f:planar_partition";
		auto planar_segments = mesh->face_property<int>(partition_name, -1);//auto 
		cur_id = Planar_FeaturePreserving::enumerate_planar_Featurepreserving(mesh, planar_segments, angle, area_threshold, fitting_threshold);

		const std::string color_name = "f:color_planar_partition";
		auto coloring = mesh->face_property<vec3>(color_name, vec3(0, 0, 0));
		Renderer::color_from_segmentation(mesh, planar_segments, coloring);
		auto faces = mesh->renderer()->get_triangles_drawable("faces");
		faces->set_property_coloring(State::FACE, color_name);


		mesh->renderer()->update();
		viewer_->update();
		window_->updateUi();
	}

}

void DialogFeaturePreservingMeshSimplification::apply_QEM()
{
	if ((viewer_->secondModel()) == nullptr)
	{
		SurfaceMesh *model_one = dynamic_cast<SurfaceMesh *>(viewer_->originModel());
		copyorigin = new SurfaceMesh(*dynamic_cast<SurfaceMesh *>(model_one));
		if (copyorigin) {
			copyorigin->set_name("originModelcopy");
			viewer_->addModel(copyorigin);
			viewer_->setCurrentModel(copyorigin);
			viewer_->update();
		}
	}
	if ((viewer_->thirdModel()) == nullptr)
	{
		SurfaceMesh *model_two = dynamic_cast<SurfaceMesh *>(viewer_->secondModel());
		copysecond = new SurfaceMesh(*dynamic_cast<SurfaceMesh *>(model_two));
		if (copysecond) {
			copysecond->set_name("secondModelcopy");
			viewer_->addModel(copysecond);
			viewer_->setCurrentModel(copysecond);
			viewer_->update();
		}
	}

	SurfaceMesh *model = dynamic_cast<SurfaceMesh *>(viewer_->thirdModel());
	
	if (!model)
		return;
	if (copythird != NULL)
	{
		viewer_->deleteModel(copythird);
		copythird = NULL;
	}
	copythird = new SurfaceMesh(*dynamic_cast<SurfaceMesh *>(model));

	if (copythird) {
		const std::string &name = file_system::name_less_extension(model->name()) + "_QEM";
		copythird->set_name(name);
		viewer_->addModel(copythird);
		viewer_->setCurrentModel(copythird);

		setQEMversion();
		SurfaceMesh *mesh = dynamic_cast<SurfaceMesh *>(viewer_->currentModel());

		SurfaceMesh_error error = SurfaceMesh_error(mesh);
		SurfaceMesh *originMesh = dynamic_cast<SurfaceMesh *>(viewer_->originModel());
		error.mean_error(originMesh);
		error.RMS_error(originMesh);
		error.Hausdorff_error(originMesh);

		mesh->renderer()->update();
		viewer_->update();
		window_->updateUi();
		
	}
}

void DialogFeaturePreservingMeshSimplification::setFacesColoring()
{
	SurfaceMesh *mesh = dynamic_cast<SurfaceMesh *>(viewer_->currentModel());
	if (checkBox_FacesColoring->isChecked())
	{
		auto faces = mesh->renderer()->get_triangles_drawable("faces");
		faces->set_property_coloring(State::FACE, "f:color_planar_partition");
	}
	else
	{
		auto faces = mesh->renderer()->get_triangles_drawable("faces");
		faces->set_coloring(State::UNIFORM_COLOR, State::FACE, "uniform color");
	}

	mesh->renderer()->update();
	viewer_->update();
	window_->updateUi();
}

void DialogFeaturePreservingMeshSimplification::setQEMversion()
{
	SurfaceMesh *mesh = dynamic_cast<SurfaceMesh *>(viewer_->currentModel());
	viewer_->setShowOnly();
	if (!mesh)
		return;
	const int points_number = BoxPointsNumber->value();

	SurfaceMeshQEM simp = SurfaceMeshQEM(mesh);
	if (checkBox_traditionalQEM->isChecked())
	{
		simp.initialize(0,10.0,0.0, 0,0.0);
		simp.simplify(points_number);
	}
	else
	{
		simp.initialize(cur_id, 10.0, 0.0, 0, 0.0);
		simp.simplify(points_number);
	}
}