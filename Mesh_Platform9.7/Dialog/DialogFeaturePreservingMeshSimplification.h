
#ifndef DIALOG_FEATURE_PRESERVING_MESH_SIMPLIFICATION_H
#define DIALOG_FEATURE_PRESERVING_MESH_SIMPLIFICATION_H

#include "ui_Dialog_FeaturePreservingMeshSimplification.h"
#include <QtWidgets\QDialog>

class Mesh_PlatformClass;
class ViewerQt;


class DialogFeaturePreservingMeshSimplification : public QDialog, public Ui::DialogFeaturePreservingMeshSimplification {
	Q_OBJECT

public:
	explicit DialogFeaturePreservingMeshSimplification(Mesh_PlatformClass *window);
	~DialogFeaturePreservingMeshSimplification();

protected:

	ViewerQt *viewer_;
	Mesh_PlatformClass *window_;
	int cur_id = 0;

private Q_SLOTS:

	void apply_BF();
	void apply_RegionGrow();
	void apply_QEM();

};

#endif