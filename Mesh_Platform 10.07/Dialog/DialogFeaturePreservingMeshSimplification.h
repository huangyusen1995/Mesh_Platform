
#ifndef DIALOG_FEATURE_PRESERVING_MESH_SIMPLIFICATION_H
#define DIALOG_FEATURE_PRESERVING_MESH_SIMPLIFICATION_H

#include "ui_Dialog_FeaturePreservingMeshSimplification.h"
#include <QtWidgets\QDialog>

class Mesh_PlatformClass;
class ViewerQt;
namespace easy3d
{
	class Model;
}


class DialogFeaturePreservingMeshSimplification : public QDialog, public Ui::DialogFeaturePreservingMeshSimplification {
	Q_OBJECT

public:
	explicit DialogFeaturePreservingMeshSimplification(Mesh_PlatformClass *window);
	~DialogFeaturePreservingMeshSimplification();

protected:

	ViewerQt *viewer_;
	Mesh_PlatformClass *window_;
	int cur_id = 0;

	easy3d::Model* copyorigin;
	easy3d::Model* copysecond;
	easy3d::Model* copythird;

private Q_SLOTS:

	void apply_BF();
	void apply_RegionGrow();
	void apply_QEM();
	void setFacesColoring();
	int setQEMversion();

};

#endif