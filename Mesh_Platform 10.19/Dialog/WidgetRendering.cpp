#include "WidgetRendering.h"
#include "..\Mesh_Platform.h"
#include "..\viewer_mesh_platform.h"

#include <QtGui/QKeyEvent>
#include <QtGui/QIntValidator>

#include <easy3d/renderer/renderer.h>
#include <easy3d/renderer/setting.h>
#include <easy3d/renderer/camera.h>
#include <easy3d/renderer/buffers.h>
#include <easy3d/renderer/clipping_plane.h>
#include <easy3d/renderer/drawable_lines.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/drawable_triangles.h>
#include <easy3d/renderer/manipulator.h>
#include <easy3d/renderer/transform.h>
#include <easy3d/renderer/texture_manager.h>
#include <easy3d/util/progress.h>
#include <easy3d/core/graph.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/core/poly_mesh.h>
#include <easy3d/fileio/resources.h>



namespace triangles_detail {

	template <typename MODEL>
	void updateNormalFieldBuffer(MODEL* model, const std::string &name) {
		if (!model)
			return;

		if (name == "f:normal") {
			auto normals = model->template get_face_property<vec3>(name);
			if (!normals)
				model->update_face_normals();
		}
		else {
			auto prop = model->template get_face_property<vec3>(name);
			if (!prop && name != "disabled") {
				LOG(ERROR) << "vector field '" << name << "' doesn't exist";
				return;
			}
		}

		// a vector field is visualized as a LinesDrawable whose name is the same as the vector field
		//一个向量场被可视化为一个名称与向量场相同的LinesDrawable
		auto drawable = model->renderer()->get_lines_drawable("vector");
		if (!drawable) {
			drawable = model->renderer()->add_lines_drawable("vector");
			drawable->set_update_func(
				[model, drawable, name](Model *m, Drawable *d) -> void {
					buffers::update(model, dynamic_cast<LinesDrawable *>(drawable), name, State::FACE, 2.0);
				}
			);
		}
	}


}

using namespace easy3d;

std::vector<WidgetRendering::ColorMaps> WidgetRendering::colormaps;
WidgetRendering::WidgetRendering(QWidget *parent)
	: QWidget(parent), ui(new Ui::WidgetRendering) {
	
	window_ = dynamic_cast<Mesh_PlatformClass *>(parent);
	viewer_ = window_->viewer();

	ui->setupUi(this);

	connect(ui->checkBox_DrawPoint, SIGNAL(toggled(bool)), this, SLOT(setPointVisible(bool)));
	connect(ui->checkBox_DrawEdge, SIGNAL(toggled(bool)), this, SLOT(setEdgeVisible(bool)));
	connect(ui->checkBox_DrawTriangle, SIGNAL(toggled(bool)), this, SLOT(setTriangleVisible(bool)));
	connect(ui->checkBox_DrawFaceNormals, SIGNAL(toggled(bool)), this, SLOT(setFaceNormalVisible(bool)));
	connect(ui->pushButton_OriginMesh, SIGNAL(clicked()), this, SLOT(original_model()));
	connect(ui->pushButton_ResultMesh, SIGNAL(clicked()), this, SLOT(result_model()));
	connect(ui->pushButton_ClearMesh, SIGNAL(clicked()), this, SLOT(clear_model()));
	connect(ui->comboBox_ScaleField, SIGNAL(currentIndexChanged(int)), this, SLOT(set_ScalarFieldStyle(const QString &)));
	connect(ui->comboBox_coloring, SIGNAL(currentIndexChanged(const QString &)), this, SLOT(set_ColorScheme(const QString &)));
	connect(ui->checkBoxScalarFieldClamp, SIGNAL(toggled(bool)), this, SLOT(setScalarFieldClamp(bool)));
	connect(ui->doubleSpinBox_ScalarFieldClampLower, SIGNAL(valueChanged(double)), this, SLOT(setScalarField_ClampLower(double)));
	connect(ui->doubleSpinBox_ScalarFieldClampUpper, SIGNAL(valueChanged(double)), this, SLOT(setScalarField_ClampUpper(double)));
}


WidgetRendering::~WidgetRendering() {
	delete ui;
}


Drawable *WidgetRendering::drawableTriangles() {
	auto model = viewer_->originModel();
	if (!model) {
		active_drawable_.clear();
		return nullptr;
	}

	auto pos = active_drawable_.find(model);
	if (pos != active_drawable_.end())
		return model->renderer()->get_triangles_drawable(pos->second);
	else {
		for (auto d : model->renderer()->triangles_drawables()) {
			active_drawable_[model] = d->name();
			return d;
		}
		return nullptr;
	}
}

void WidgetRendering::setPointVisible(bool b)
{
	QKeyEvent *e;	
	e=new QKeyEvent(QEvent::KeyPress, Qt::Key_V,0 , "86", false,  1);
	viewer_->keyPressEvent(e);
}

void WidgetRendering::setEdgeVisible(bool b)
{
	QKeyEvent *e;
	e = new QKeyEvent(QEvent::KeyPress, Qt::Key_E, 0, "69", false, 1);
	viewer_->keyPressEvent(e);
}

void WidgetRendering::setTriangleVisible(bool b)
{
	auto model = viewer_->currentModel();
	auto *triangles = model->renderer()->get_triangles_drawable("faces");
	if (triangles) {
		triangles->set_visible(!triangles->is_visible());
	}
	viewer_->update();
}

void WidgetRendering::setFaceNormalVisible(bool b)
{
	auto model = viewer_->currentModel();
	//updateNormalFieldBuffer(mesh, "f:normal");
	if (dynamic_cast<SurfaceMesh *>(model)) {
		auto mesh = dynamic_cast<SurfaceMesh *>(model);
		triangles_detail::updateNormalFieldBuffer(mesh, "f:normal");
	}
	else if (dynamic_cast<PolyMesh *>(model)) {
		auto mesh = dynamic_cast<PolyMesh *>(model);
		triangles_detail::updateNormalFieldBuffer(mesh, "f:normal");
	}

	auto d = model->renderer()->get_lines_drawable("vector");
	if (d) 
	{ 
		d->set_visible(b);
	}

	viewer_->update();
	
}

void WidgetRendering::original_model()
{	
	Model* original_model = viewer_->originModel();
	if (original_model)
	{
		std::cout << "now" << std::endl;
		std::string original_model_name = original_model->name();
		std::cout << "original model name:" << original_model_name << "\n";
		for (auto m : viewer_->models()) {
			m->renderer()->set_visible(m == original_model);
		}
	}
	viewer_->update();
	window_->updateUi();
}

void WidgetRendering::result_model()
{
	Model* result_model = viewer_->resultModel();
	if (result_model)
	{
		std::cout << "now" << std::endl;
		std::string result_model_name = result_model->name();
		std::cout << "result model name:" << result_model_name << "\n";
		for (auto m : viewer_->models()) {
			m->renderer()->set_visible(m == result_model);
		}
	}
	
	viewer_->update();
	window_->updateUi();
}

void WidgetRendering::clear_model()
{
	for (auto m : viewer_->models()) {
		if (m) {
			viewer_->deleteModel(viewer_->currentModel());
		}
		else
			return;
	}
	
	viewer_->update();
	window_->updateUi();
}

void WidgetRendering::set_ScalarFieldStyle(const QString &text1)
{
	SurfaceMesh *original_mesh = dynamic_cast<SurfaceMesh *>(viewer_->originModel());
	//SurfaceMesh *original_mesh = dynamic_cast<SurfaceMesh *>(viewer_->currentModel());
	auto faces = original_mesh->renderer()->get_triangles_drawable("faces");
	std::string textureFile;
	if (text1 == "rainbow")
	{
		textureFile = resource::directory() + "/colormaps/rainbow.png";
	}
	else if (text1 == "default")
	{
		textureFile = resource::directory() + "/colormaps/default.png";
	}
	Texture *texture = TextureManager::request(textureFile);
	if (!texture) {
		LOG(ERROR) << "Error: failed to create texture.";
		return;
	}
	faces->set_texture(texture);
}

void WidgetRendering::set_ColorScheme(const QString &text2)
{
	SurfaceMesh *original_mesh = dynamic_cast<SurfaceMesh *>(viewer_->originModel());
	SurfaceMesh *current_mesh = dynamic_cast<SurfaceMesh *>(viewer_->currentModel());
	//SurfaceMesh* ground_truth= dynamic_cast<SurfaceMesh *>(viewer_->originModel());
	if (text2 == "uniform color")
	{
		auto faces = original_mesh->renderer()->get_triangles_drawable("faces");
		faces->set_coloring(State::UNIFORM_COLOR, State::FACE, "uniform color");
	}
	else if (text2 == "scalar-v:min_distance")
	{
		//double m_b = -0.005;
		auto min_distance = original_mesh->get_vertex_property<double>("v:min_distance");
		
		auto faces = original_mesh->renderer()->get_triangles_drawable("faces");
		faces->set_scalar_coloring(State::VERTEX, "v:min_distance");
		//faces->set_scalar_coloring(State::FACE, "v:min_distance");
		/*auto vertices = original_mesh->renderer()->get_points_drawable("vertices");
		vertices->set_scalar_coloring(State::VERTEX, "v:min_distance");*/

		set_ScalarFieldStyle(ui->comboBox_ScaleField->currentText());
		original_mesh->renderer()->update();
	}
	for (auto m : viewer_->models()) {
		m->renderer()->set_visible(m == original_mesh);
	}
	viewer_->update();
	window_->updateUi();
		
}

void WidgetRendering::setScalarFieldClamp(bool b) {
	auto d = drawableTriangles();
	d->set_clamp_range(b);
	d->update();
	viewer_->update();
}

void WidgetRendering::setScalarField_ClampLower(double v) {
	auto d = drawableTriangles();
	if (d->clamp_upper() * 100 + v < 100) {
		d->set_clamp_lower(v / 100.0f);
		d->update();
		viewer_->update();
	}
	else
		LOG(WARNING) << "invalid clamp range (the sum of lower and upper must be smaller than 100)";
}

void WidgetRendering::setScalarField_ClampUpper(double v) {
	auto d = drawableTriangles();
	if (d->clamp_lower() * 100 + v < 100) {
		d->set_clamp_upper(v / 100.0f);
		d->update();
		viewer_->update();
	}
	else
		LOG(WARNING) << "invalid clamp range (the sum of lower and upper must be smaller than 100)";
}












