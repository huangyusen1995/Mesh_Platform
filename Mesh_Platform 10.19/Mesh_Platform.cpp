#include "Mesh_Platform.h"
#include <ui_Mesh_Platform.h>
#include <iostream>
#include <string>
/*
#include <QFileDialog>
#include <QDropEvent>
#include <QMimeData>
#include <QSettings>
#include <QMessageBox>
#include <QColorDialog>
*/
#include <QtWidgets\QFileDialog>
#include <QtGui\QDropEvent>
#include <QtCore\QMimeData>
#include <QtCore\QSettings>
#include <QtWidgets\QMessageBox>
#include <QtWidgets\QColorDialog>
#include <QtWidgets\QDesktopWidget>

#include <easy3d/core/surface_mesh.h>
#include <easy3d/core/graph.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/core/poly_mesh.h>
#include <easy3d/fileio/point_cloud_io.h>
#include <easy3d/fileio/graph_io.h>
#include <easy3d/fileio/surface_mesh_io.h>
#include <easy3d/fileio/poly_mesh_io.h>
#include <easy3d/fileio/ply_reader_writer.h>
#include <easy3d/fileio/point_cloud_io_ptx.h>
#include <easy3d/fileio/resources.h>
#include <easy3d/util/file_system.h>
#include <easy3d/util/logging.h>
#include <easy3d/util/progress.h>
#include <easy3d/algo/surface_mesh_components.h>
#include <easy3d/algo/surface_mesh_topology.h>
#include <easy3d/renderer/renderer.h>
#include <easy3d/renderer/drawable_triangles.h>
#include <easy3d/renderer/texture_manager.h>


#include "Dialog/DialogFeaturePreservingMeshSimplification.h"
#include "Dialog/DialogMSAT_View.h"
#include "Dialog/WidgetRendering.h"
#include "viewer_mesh_platform.h"
#include "Algorithm/SurfaceMesh_error.h"


using namespace easy3d;

Mesh_PlatformClass::Mesh_PlatformClass(QWidget *parent)
	: QMainWindow(parent)
	,ui(new Ui::Mesh_PlatformClass)
{
	ui->setupUi(this);
	viewer_ = new ViewerQt(this);
	connect(viewer_, SIGNAL(currentModelChanged()), this, SLOT(onCurrentModelChanged()));
	setCentralWidget(viewer_);
	const int width = 400; 
	ui->dockWidgetGeometric->setFixedWidth(width);

	WidgetRendering_ = new WidgetRendering(this);
	const int width_rendering = 380;
	WidgetRendering_->setFixedWidth(width_rendering);
	ui->verticalLayoutRendering->addWidget(WidgetRendering_);

	createActions();

	setFocusPolicy(Qt::StrongFocus);
	setContextMenuPolicy(Qt::CustomContextMenu);
	setAcceptDrops(true);

	//setBaseSize(1280, 960);
	    setWindowState(Qt::WindowMaximized);

	readSettings();
	updateWindowTitle();
}

Mesh_PlatformClass::~Mesh_PlatformClass()
{
}

void Mesh_PlatformClass::dragEnterEvent(QDragEnterEvent *e) {
	if (e->mimeData()->hasUrls())
		e->acceptProposedAction();
}

void Mesh_PlatformClass::dropEvent(QDropEvent *e) {
	if (e->mimeData()->hasUrls())
		e->acceptProposedAction();

	int count = 0;
	foreach(const QUrl &url, e->mimeData()->urls()) {
		const QString &fileName = url.toLocalFile();
		if (open(fileName.toStdString()))
			++count;
	}

	if (count > 0)
		viewer_->update();
}

bool Mesh_PlatformClass::onOpen() {
	const QStringList& fileNames = QFileDialog::getOpenFileNames(
		this,
		"Open file(s)",
		curDataDirectory_,
		"Supported formats (*.ply *.obj *.off *.stl *.sm *.bin *.las *.laz *.xyz *.bxyz *.vg *.bvg *.ptx *.plm *.pm *.mesh)\n"
		"Surface Mesh (*.ply *.obj *.off *.stl *.sm)\n"
		"Point Cloud (*.ply *.bin *.ptx *.las *.laz *.xyz *.bxyz *.vg *.bvg *.ptx)\n"
		"Polyhedral Mesh (*.plm *.pm *.mesh)\n"
		"All formats (*.*)"
	);

	// Hide closed dialog隐藏对话框关闭
	QApplication::processEvents();

	if (fileNames.empty())
		return false;

	int count = 0;
	ProgressLogger progress(fileNames.size(), false, false);
	for (const auto& name : fileNames) {
		if (progress.is_canceled()) {
			LOG(WARNING) << "opening files cancelled";
			break;
		}
		if (open(name.toStdString()))
			++count;
		progress.next();
	}
	if (count > 0)
		viewer_->update();

	return count > 0;
}



bool Mesh_PlatformClass::onSave() {
	const Model* model = viewer_->currentModel();
	if (!model) {
		std::cerr << "no model exists" << std::endl;
		return false;
	}

	std::string default_file_name = model->name();
	if (file_system::extension(default_file_name).empty()) // no extension?
		default_file_name += ".ply"; // default to ply

	const QString& fileName = QFileDialog::getSaveFileName(
		this,
		"Open file(s)",
		QString::fromStdString(default_file_name),
		"Supported formats (*.ply *.obj *.off *.stl *.sm *.bin *.las *.laz *.xyz *.bxyz *.vg *.bvg *.plm *.pm *.mesh)\n"
		"Surface Mesh (*.ply *.obj *.off *.stl *.sm)\n"
		"Point Cloud (*.ply *.bin *.ptx *.las *.laz *.xyz *.bxyz *.vg *.bvg)\n"
		"Polyhedral Mesh (*.plm *.pm *.mesh)\n"
		"All formats (*.*)"
	);

	if (fileName.isEmpty())
		return false;

	bool saved = false;
	if (dynamic_cast<const PointCloud*>(model)) {
		const PointCloud* cloud = dynamic_cast<const PointCloud*>(model);
		saved = PointCloudIO::save(fileName.toStdString(), cloud);
	}
	else if (dynamic_cast<const SurfaceMesh*>(model)) {
		const SurfaceMesh* mesh = dynamic_cast<const SurfaceMesh*>(model);
		saved = SurfaceMeshIO::save(fileName.toStdString(), mesh);
	}
	else if (dynamic_cast<const Graph*>(model)) {
		const Graph* graph = dynamic_cast<const Graph*>(model);
		saved = GraphIO::save(fileName.toStdString(), graph);
	}

	if (saved) {
		std::cout << "model successfully saved to: " << fileName.toStdString();
		setCurrentFile(fileName);
		return true;
	}

	return false;
}

Model* Mesh_PlatformClass::open(const std::string& file_name) {
	auto models = viewer_->models();
	for (auto m : models) {
		if (m->name() == file_name) {
			LOG(WARNING) << "model already loaded: " << file_name;
			return nullptr;
		}
	}

	const std::string& ext = file_system::extension(file_name, true);
	bool is_ply_mesh = false;
	if (ext == "ply")
		is_ply_mesh = (io::PlyReader::num_instances(file_name, "face") > 0);
	
	//opened_model = nullptr;
	Model* model = nullptr;
	if ((ext == "ply" && is_ply_mesh) || ext == "obj" || ext == "off" || ext == "stl" || ext == "sm" || ext == "plg") { // mesh
		model = SurfaceMeshIO::load(file_name);
	}
	else if (ext == "ply" && io::PlyReader::num_instances(file_name, "edge") > 0) {
		model = GraphIO::load(file_name);
	}
	else if (ext == "plm" || ext == "pm" || ext == "mesh") {
		model = PolyMeshIO::load(file_name);
	}
	else { // point cloud
		if (ext == "ptx") {
			io::PointCloudIO_ptx serializer(file_name);
			PointCloud* cloud = nullptr;
			while ((cloud = serializer.load_next())) {
				viewer_->addModel(cloud);
			}
		}
		else
			model = PointCloudIO::load(file_name);
	}

	if (model) {
		model->set_name(file_name);
		viewer_->addModel(model);
		setCurrentFile(QString::fromStdString(file_name));
		//opened_model = model;
	}
	
	
	return model;
}


void Mesh_PlatformClass::onCurrentModelChanged() {
	const Model* m = viewer_->currentModel();
	if (m) {
		viewer_->fitScreen(m);

		const std::string& name = m->name();
		setCurrentFile(QString::fromStdString(name));
	}
	else
		updateWindowTitle();
}


void Mesh_PlatformClass::setCurrentFile(const QString &fileName)
{
	QString dir = fileName.left(fileName.lastIndexOf("/"));
	if (!dir.isEmpty() && file_system::is_directory(dir.toStdString()))
		curDataDirectory_ = dir;

	setWindowModified(false);

	if (!fileName.isEmpty()) {
		recentFiles_.removeAll(fileName);
		recentFiles_.prepend(fileName);
		updateRecentFileActions();
	}

	updateWindowTitle();
}


void Mesh_PlatformClass::onOpenRecentFile()
{
	if (okToContinue()) {
		QAction *action = qobject_cast<QAction *>(sender());
		if (action) {
			const QString filename(action->data().toString());
			if (open(filename.toStdString()))
				viewer_->update();
		}
	}
}


void Mesh_PlatformClass::onClearRecentFiles() {
	recentFiles_.clear();
	updateRecentFileActions();
}


void Mesh_PlatformClass::saveSnapshot() {
	const Model* model = viewer_->currentModel();

	const bool overwrite = false;
	std::string default_file_name("untitled.png");
	if (model)
		default_file_name = file_system::replace_extension(model->name(), "png");
	QString proposedFormat = "PNG (*.png)";
	const QString fileName = QFileDialog::getSaveFileName(
		this,
		"Please choose a file name",
		QString::fromStdString(default_file_name),
		"Image Files (*.png *.jpg *.bmp *.ppm)\n"
		"PNG (*.png)\n"
		"JPG (*.jpg)\n"
		"Windows Bitmap (*.bmp)\n"
		"24bit RGB Bitmap (*.ppm)\n"
		"All Files (*.*)",
		nullptr,
		overwrite ? QFileDialog::DontConfirmOverwrite : QFlags<QFileDialog::Option>(nullptr)
	);
	// Hide closed dialog
	QApplication::processEvents();

	if (fileName.isEmpty())
		return;

	viewer_->saveSnapshot(fileName);
}


void Mesh_PlatformClass::setBackgroundColor() {
	const vec4& c = viewer_->backGroundColor();
	QColor orig(static_cast<int>(c.r * 255), static_cast<int>(c.g * 255), static_cast<int>(c.b * 255), static_cast<int>(c.a * 255));
	const QColor& color = QColorDialog::getColor(orig, this);
	if (color.isValid()) {
		const vec4 newColor(color.redF(), color.greenF(), color.blueF(), color.alphaF());
		viewer_->setBackgroundColor(newColor);
		viewer_->update();
	}
}


bool Mesh_PlatformClass::okToContinue()
{
	if (isWindowModified()) {
		int r = QMessageBox::warning(this, tr("ViewerQt"),
			tr("The model has been modified.\n"
				"Do you want to save your changes?"),
			QMessageBox::Yes | QMessageBox::Default,
			QMessageBox::No,
			QMessageBox::Cancel | QMessageBox::Escape);
		if (r == QMessageBox::Yes)
			return onSave();
		else if (r == QMessageBox::Cancel)
			return false;
	}
	return true;
}


void Mesh_PlatformClass::onAbout()
{
	QString title = QMessageBox::tr("<h3>About</h3>");

	QString text = QMessageBox::tr(
		"<p>This platform is mainly used for mesh reconstruction</p>"
		"<p>CUG -- Geometric optimization team.<br>"
		//"<a href=\"mailto:liangliang.nan@gmail.com\">liangliang.nan@gmail.com</a><br>"
		//"<a href=\"https://3d.bk.tudelft.nl/liangliang/\">https://3d.bk.tudelft.nl/liangliang/</a></p>"
	);

	//QMessageBox::about(this, title, text);
	QMessageBox::about(this, "About Platform", title + text);
}


void Mesh_PlatformClass::readSettings()
{
	QSettings settings("liangliang.nan@gmail.com", "ViewerQt");
	recentFiles_ = settings.value("recentFiles").toStringList();
	updateRecentFileActions();
	curDataDirectory_ = settings.value("currentDirectory").toString();
}


void Mesh_PlatformClass::writeSettings()
{
	QSettings settings("liangliang.nan@gmail.com", "ViewerQt");
	settings.setValue("recentFiles", recentFiles_);
	if (!curDataDirectory_.isEmpty() && file_system::is_directory(curDataDirectory_.toStdString()))
		settings.setValue("currentDirectory", curDataDirectory_);
}


void Mesh_PlatformClass::updateWindowTitle() {
	Model* model = viewer_->currentModel();

#ifndef NDEBUG
	QString title = "ViewerQt (Debug Version)";
#else
	QString title = "Mesh_Platform";
#endif // _DEBUG

	QString fileName("Untitled");
	if (model)
		fileName = QString::fromStdString(model->name());

	title = tr("%1[*] - %2").arg(strippedName(fileName)).arg(title);
	setWindowTitle(title);
}


void Mesh_PlatformClass::closeEvent(QCloseEvent *event)
{
	if (okToContinue()) {
		writeSettings();
		event->accept();
	}
	else {
		event->ignore();
	}
}


void Mesh_PlatformClass::updateRecentFileActions()
{
	QMutableStringListIterator i(recentFiles_);
	while (i.hasNext()) {
		if (!QFile::exists(i.next()))
			i.remove();
	}

	for (int j = 0; j < MaxRecentFiles; ++j) {
		if (j < recentFiles_.count()) {
			QString text = tr("&%1 %2").arg(j + 1).arg(strippedName(recentFiles_[j]));
			actionsRecentFile[j]->setText(text);
			actionsRecentFile[j]->setData(recentFiles_[j]);
			actionsRecentFile[j]->setVisible(true);
		}
		else {
			actionsRecentFile[j]->setVisible(false);
		}
	}

	actionSeparator->setVisible(!recentFiles_.isEmpty());
}

void Mesh_PlatformClass::updateUi() {
	const Model* model = viewer_->currentModel();
	if (model) {
		const std::string& name = model->name();
		setCurrentFile(QString::fromStdString(name));
	}
	else
		updateWindowTitle();

	//updateRenderingPanel();//更新渲染面板
	//updateStatusBar();//更新状态栏
}


QString Mesh_PlatformClass::strippedName(const QString &fullFileName)
{
	return QFileInfo(fullFileName).fileName();
}


void Mesh_PlatformClass::createActions() {
	// file menu
	createActionsForFileMenu();

	// view menu
	createActionsForViewMenu();

	// topology menu
	createActionsForTopologyMenu();

	//Alogrithm menu
	createActionsForAlgorithmMenu();

	//error menu
	createActionsForErrorMenu();
	// about menu
	connect(ui->actionAbout_Mesh_Platform, SIGNAL(triggered()), this, SLOT(onAbout()));
}


void Mesh_PlatformClass::createActionsForFileMenu() {
	connect(ui->actionOpen, SIGNAL(triggered()), this, SLOT(onOpen()));
	connect(ui->actionSave, SIGNAL(triggered()), this, SLOT(onSave()));
	//connect(ui->actionLoad_GroundTruth, SIGNAL(triggered()), this, SLOT(onLoadGroundTruth()));
	actionSeparator = ui->menuFile->addSeparator();

	QList<QAction*> actions;
	for (int i = 0; i < MaxRecentFiles; ++i) {
		actionsRecentFile[i] = new QAction(this);
		actionsRecentFile[i]->setVisible(false);
		connect(actionsRecentFile[i], SIGNAL(triggered()), this, SLOT(onOpenRecentFile()));

		actions.push_back(actionsRecentFile[i]);
	}
	ui->menuRecent_Files->insertActions(ui->actionClearRecentFiles, actions);
	ui->menuRecent_Files->insertSeparator(ui->actionClearRecentFiles);
	connect(ui->actionClearRecentFiles, SIGNAL(triggered()), this, SLOT(onClearRecentFiles()));

	connect(ui->actionExit, SIGNAL(triggered()), this, SLOT(close()));
	ui->actionExit->setShortcut(QString("Ctrl+Q"));
}


void Mesh_PlatformClass::createActionsForViewMenu() {
	connect(ui->actionSnapshot, SIGNAL(triggered()), this, SLOT(saveSnapshot()));

	ui->menuView->addSeparator();

	connect(ui->actionBackground_Color, SIGNAL(triggered()), this, SLOT(setBackgroundColor()));
}


void Mesh_PlatformClass::createActionsForTopologyMenu() {
	connect(ui->actionShow_Statistics, SIGNAL(triggered()), this, SLOT(reportTopologyStatistics()));
}

void Mesh_PlatformClass::createActionsForAlgorithmMenu() {
	
	connect(ui->actionFeature_preserving_3D_mesh_simplification, SIGNAL(triggered()), this, SLOT(FeaturePreservingMeshSimplification()));
	connect(ui->actionMumford_Shah_AT, SIGNAL(triggered()), this, SLOT(MSAT()));

}

void Mesh_PlatformClass::createActionsForErrorMenu() {
	connect(ui->actionMean_Error_2, SIGNAL(triggered()), this, SLOT(runMeanError()));

}

void Mesh_PlatformClass::reportTopologyStatistics() {
	SurfaceMesh *mesh = dynamic_cast<SurfaceMesh *>(viewer()->currentModel());
	if (!mesh)
		return;

	const std::string simple_name = file_system::simple_name(mesh->name());
	if (simple_name.empty())
		std::cout << "#elements in model (with unknown name): ";
	else
		std::cout << "#elements in model '" << file_system::simple_name(mesh->name()) << "': ";

	std::cout << "#face = " << mesh->n_faces() << ", #vertex = " << mesh->n_vertices() << ", #edge = "
		<< mesh->n_edges() << std::endl;

	// count isolated vertices
	std::size_t count = 0;
	for (auto v : mesh->vertices()) {
		if (mesh->is_isolated(v))
			++count;
	}
	if (count > 0)
		std::cout << "#isolated vertices: " << count << std::endl;

	const auto &components = SurfaceMeshComponent::extract(mesh);
	std::cout << "#connected component: " << components.size() << std::endl;

	const std::size_t num = 10;
	if (components.size() > num)
		std::cout << "\ttopology of the first " << num << " components:" << std::endl;

	for (std::size_t i = 0; i < std::min(components.size(), num); ++i) {
		const SurfaceMeshComponent& comp = components[i];
		SurfaceMeshTopology topo(&comp);
		std::string type = "unknown";
		if (topo.is_sphere())
			type = "sphere";
		else if (topo.is_disc())
			type = "disc";
		else if (topo.is_cylinder())
			type = "cylinder";
		else if (topo.is_torus())
			type = "torus";
		else if (topo.is_closed())
			type = "unknown closed";

		std::cout << "\t\t" << i << ": "
			<< type
			<< ", #face = " << comp.n_faces() << ", #vertex = " << comp.n_vertices() << ", #edge = " << comp.n_edges()
			<< ", #border = " << topo.number_of_borders();
		if (topo.number_of_borders() == 1)
			std::cout << ", border size = " << topo.largest_border_size();
		else if (topo.number_of_borders() > 1)
			std::cout << ", largest border size = " << topo.largest_border_size();
		std::cout << std::endl;
	}
}


void Mesh_PlatformClass::FeaturePreservingMeshSimplification()
{
	static DialogFeaturePreservingMeshSimplification* dialog = nullptr;
	if (!dialog)
		dialog = new DialogFeaturePreservingMeshSimplification(this);
	dialog->show();

}

void Mesh_PlatformClass::MSAT()
{
	static DialogMSAT_View* dialog = nullptr;
	if (!dialog)
		dialog = new DialogMSAT_View(this);
	dialog->show();
}


void Mesh_PlatformClass::runMeanError()
{
	SurfaceMesh *original_mesh = dynamic_cast<SurfaceMesh *>(viewer_->originModel());
	SurfaceMesh *result_mesh = dynamic_cast<SurfaceMesh *>(viewer_->currentModel());
	SurfaceMesh_error error = SurfaceMesh_error(result_mesh);
	error.mean_error(original_mesh);
	error.RMS_error(original_mesh);
	error.Hausdorff_error(original_mesh);
	error.SAMD_mean_error(original_mesh);
}