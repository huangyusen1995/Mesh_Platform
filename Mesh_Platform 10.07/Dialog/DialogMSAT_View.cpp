#include "DialogMSAT_View.h"

#include "..\Algorithm\MusfordShahAT.h"

#include <easy3d/renderer/renderer.h>
#include <easy3d/renderer/drawable_lines.h>
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


	const Qt::WindowFlags flags = windowFlags();
	setWindowFlags(flags | Qt::Tool);
	setupUi(this);
	connect(pushButtonRunMSAT, SIGNAL(clicked()), this, SLOT(apply_MSAT()));

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

