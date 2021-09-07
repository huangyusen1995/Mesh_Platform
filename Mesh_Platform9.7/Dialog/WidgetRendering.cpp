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
#include <easy3d/util/progress.h>
#include <easy3d/core/graph.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/core/poly_mesh.h>


using namespace easy3d;

WidgetRendering::WidgetRendering(QWidget *parent)
	: QWidget(parent), ui(new Ui::WidgetRendering) {
	
	window_ = dynamic_cast<Mesh_PlatformClass *>(parent);
	viewer_ = window_->viewer();

	//const Qt::WindowFlags flags = windowFlags();
	//setWindowFlags(flags | Qt::Tool);

	//setWindowFlags(Qt::CustomizeWindowHint | Qt::FramelessWindowHint);
	//hide();
	ui->setupUi(this);
	//QValidator *validator = new QIntValidator(1.0,1000.0, this);
	//lineEdit_EdgeWidth->setValidator(validator);

	
	connect(ui->checkBox_DrawPoint, SIGNAL(toggled(bool)), this, SLOT(setPointVisible(bool)));
	connect(ui->checkBox_DrawEdge, SIGNAL(toggled(bool)), this, SLOT(setEdgeVisible(bool)));
	connect(ui->checkBox_DrawTriangle, SIGNAL(toggled(bool)), this, SLOT(setTriangleVisible(bool)));
	connect(ui->checkBox_DrawFaceNormals, SIGNAL(toggled(bool)), this, SLOT(setFaceNormalVisible(bool)));
	connect(ui->pushButton_OriginMesh, SIGNAL(clicked()), this, SLOT(original_model()));

}


WidgetRendering::~WidgetRendering() {
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
	//lineEdit_EdgeWidth->setText(QString("1.0"));
	//const float edges_width = lineEdit_EdgeWidth->text().toFloat();
	//auto model = viewer_->currentModel();
	//if (model) {
	//	auto *edges = model->renderer()->get_lines_drawable("edges");
	//	if (!edges) {
	//		if (!dynamic_cast<PointCloud *>(model)) { // no default "edges" drawable for point clouds
	//			edges = model->renderer()->add_lines_drawable("edges");
	//			if (dynamic_cast<SurfaceMesh *>(model)) {
	//				edges->set_uniform_coloring(setting::surface_mesh_edges_color);
	//				edges->set_line_width(edges_width);
	//			}
	//			else if (dynamic_cast<Graph *>(model)) {
	//				edges->set_uniform_coloring(setting::graph_edges_color);
	//				edges->set_line_width(edges_width);
	//				edges->set_impostor_type(LinesDrawable::CYLINDER);
	//			}
	//		}
	//	}
	//	else
	//		edges->set_visible(!edges->is_visible());
	//}

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


namespace triangles_detail {

	template<typename MODEL>
	// spinbox must appear as a parameter because its value maybe changed and its is used later in a lambda function
	void updateNormalFieldBuffer(MODEL *model, const std::string &name) {
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
					buffers::update(model, dynamic_cast<LinesDrawable *>(drawable), name, State::FACE,2.0);
				}
			);
		}
	}

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
	
	std::cout << "now"<<"\n"<<std::endl;
	/*
	std::string original_model_name = viewer_->models_[viewer_->model_idx_]->name();
	std::cout << "original_model_name:" <<original_model_name <<"\n";

	Model* original_model = window_->open(original_model_name);
	std::cout << "original_model:" << original_model<< "\n";
	viewer_->update();
	window_->updateUi();*/
}










