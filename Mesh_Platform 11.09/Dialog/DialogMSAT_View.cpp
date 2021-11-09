#include "DialogMSAT_View.h"

#include "..\Algorithm\MusfordShahAT.h"
#include "..\Algorithm\Feature_Graph.h"

#include <easy3d/renderer/renderer.h>
#include <easy3d/renderer/drawable_lines.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/setting.h>
#include <easy3d/viewer/viewer.h>
#include <easy3d/renderer/buffers.h>
#include <easy3d/util/file_system.h>


#include "..\Mesh_Platform.h"
#include "..\viewer_mesh_platform.h"

using namespace easy3d;
DialogMSAT_View::DialogMSAT_View(Mesh_PlatformClass *window) : QDialog(window)
{
	window_ = window;
	viewer_ = window_->viewer();


	copyorigin = nullptr;
	copysecond = nullptr;

	const Qt::WindowFlags flags = windowFlags();
	setWindowFlags(flags | Qt::Tool);
	setupUi(this);
	connect(pushButtonRunMSAT, SIGNAL(clicked()), this, SLOT(apply_MSAT()));
	connect(checkBox_showFeatureEdge, SIGNAL(toggled(bool)), this, SLOT(setShowFeatureEdges()));
	connect(checkBox_showFeatureVertex, SIGNAL(toggled(bool)), this, SLOT(setShowFeaturevertices()));
	connect(pushButtonFeatureGraph, SIGNAL(clicked()), this, SLOT(apply_FeatureGraph()));

}

DialogMSAT_View::~DialogMSAT_View()
{
}

void DialogMSAT_View::apply_MSAT()
{
	SurfaceMesh *model = dynamic_cast<SurfaceMesh *>(viewer_->originModel());
	if (!model)
	{
		std::cout << "Please load the original model first" << std::endl;
		return;
	}
	for (auto m : viewer_->models()) {
		if (m)
		{
			viewer_->deleteModel(viewer_->resultModel());
		}
	}

	copyorigin = new SurfaceMesh(*dynamic_cast<SurfaceMesh *>(model));

	if (copyorigin) {
		const std::string &name = file_system::name_less_extension(model->name()) + "_MSAT";
		copyorigin->set_name(name);
		viewer_->addModel(copyorigin);
		viewer_->setCurrentModel(copyorigin);
		SurfaceMesh *mesh = dynamic_cast<SurfaceMesh *>(viewer_->currentModel());
		viewer_->setShowOnly();

		if (!mesh)
			return;

		const double alpha = BoxMSATalpha->value();
		const double Threshold = BoxMSATthreshold->value();
		const double beta = BoxMSATbeta->value();
		const double epsilon = BoxMSATepsilon->value();
		const int normItrNum = BoxMSATNormItr->value();
		const int verItrNum = BoxMSATVerItr->value();
		MumfordShah_AT MSATFilter = MumfordShah_AT(mesh);
		MSATFilter.setParams(alpha, beta, Threshold, epsilon, normItrNum, verItrNum);
		MSATFilter.filtering();

		mesh->renderer()->update();
		viewer_->update();
		window_->updateUi();
	}

}

//运行feature graph程序
void DialogMSAT_View::apply_FeatureGraph()
{
	//添加代码。。。。。
	SurfaceMesh *model = dynamic_cast<SurfaceMesh *>(viewer_->secondModel());
	if (!model)
	{
		std::cout << "Please load the second Model first" << std::endl;
		return;
	}
	for (auto m : viewer_->models()) {
		if (m)
		{
			viewer_->deleteModel(viewer_->thirdModel());
		}
	}

	copysecond = new SurfaceMesh(*dynamic_cast<SurfaceMesh *>(model));

	if (copysecond) {
		const std::string &name = file_system::name_less_extension(model->name()) + "_MSAT";
		copysecond->set_name(name);
		viewer_->addModel(copysecond);
		viewer_->setCurrentModel(copysecond);
		SurfaceMesh *mesh = dynamic_cast<SurfaceMesh *>(viewer_->currentModel());
		viewer_->setShowOnly();

		if (!mesh)
			return;
		FeatherGraph_CurveGenerate FGCG = FeatherGraph_CurveGenerate(mesh);
		FGCG.vertex_select();
		FGCG.generate_curve();
	
		mesh->renderer()->update();
		viewer_->update();
		window_->updateUi();
	}




}

namespace buffer {
	template<typename MODEL>
	void update_mesh_featureborders(MODEL  *model, LinesDrawable *drawable) {
		if (model->empty()) {
			LOG(WARNING) << "model has no valid geometry";
			return;
		}

		auto prop = model->get_vertex_property<vec3>("v:point");
		auto feature = model->edge_property<bool>("e:feature", false);
		std::vector<vec3> points;
		points.reserve(model->n_edges() * 2);
		for (auto e : model->edges()) {
			if (feature[e] == true) {
				points.push_back(prop[model->vertex(e, 0)]);
				points.push_back(prop[model->vertex(e, 1)]);
			}
		}
		drawable->update_vertex_buffer(points);
		// We will draw the normal vectors in a uniform green color
		drawable->set_uniform_coloring(vec4(0.0f, 1.0f, 0.0f, 1.0f));
		// Set the line width
		drawable->set_line_width(3.0f);

	}
	template<typename MODEL>
	void update_mesh_featurevertices(MODEL  *model, PointsDrawable *drawable) {
		if (model->empty()) {
			LOG(WARNING) << "model has no valid geometry";
			return;
		}

		auto prop = model->get_vertex_property<vec3>("v:point");
		auto featureV = model->vertex_property<bool>("v:feature", false);
		std::vector<vec3> points;
		points.reserve(model->n_vertices());
	
		for (auto v : model->vertices()) {
			if (featureV[v] == true) {
				points.push_back(prop[v]);
			}
		}
		drawable->update_vertex_buffer(points);
		// We will draw the vertices in red.
		drawable->set_uniform_coloring(vec4(1.0f, 0.0f, 0.0f, 1.0f));
		// Draw the vertices as spheres.
		drawable->set_impostor_type(PointsDrawable::SPHERE);
		// Set the vertices size to 10 pixels.
		drawable->set_point_size(8);

	}

}

void DialogMSAT_View::updateVectorFieldBuffer(Model *model, const std::string &name) {
	// a vector field is visualized as a LinesDrawable whose name is the same as the vector field
	auto drawable = model->renderer()->get_lines_drawable("vector - " + name);
	if (!drawable) {
		drawable = model->renderer()->add_lines_drawable("vector - " + name);
		drawable->set_update_func([&, name](Model *m, Drawable *d) -> void {
			if (dynamic_cast<SurfaceMesh *>(m))
				buffer::update_mesh_featureborders(dynamic_cast<SurfaceMesh *>(m), dynamic_cast<LinesDrawable*>(d));
			});
	}
}

void DialogMSAT_View::updateVertexFieldBuffer(Model *model, const std::string &name)
{
	auto drawable_v = model->renderer()->get_points_drawable("points - " + name);
	if (!drawable_v) {
		drawable_v = model->renderer()->add_points_drawable("points - " + name);
		drawable_v->set_update_func([&, name](Model *m, Drawable *d) -> void {
			if (dynamic_cast<SurfaceMesh *>(m))
				buffer::update_mesh_featurevertices(dynamic_cast<SurfaceMesh *>(m), dynamic_cast<PointsDrawable*>(d));
			});
	}
}



void DialogMSAT_View::setShowFeatureEdges()
{
	std::string name = "e:feature";
	auto model = viewer_->currentModel();
	if (dynamic_cast<SurfaceMesh *>(model)) {
		auto mesh = dynamic_cast<SurfaceMesh *>(model);
		updateVectorFieldBuffer(mesh, name);
	}

	auto d = model->renderer()->get_lines_drawable("vector - " + name);
	if (checkBox_showFeatureEdge->isChecked())
	{
		d->set_visible(true);
	}
	else
	{
		d->set_visible(false);
	}
	viewer_->update();


}

void DialogMSAT_View::setShowFeaturevertices()
{
	std::string name = "v:feature";
	auto model = viewer_->currentModel();
	if (dynamic_cast<SurfaceMesh *>(model)) {
		auto mesh = dynamic_cast<SurfaceMesh *>(model);
		updateVertexFieldBuffer(mesh, name);
	}
	auto d_V = model->renderer()->get_points_drawable("points - " + name);
	if (checkBox_showFeatureVertex->isChecked())
	{
		d_V->set_visible(true);
	}
	else
	{
		d_V->set_visible(false);
	}
	viewer_->update();


}