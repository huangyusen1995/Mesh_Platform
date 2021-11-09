#pragma once
#include "ui_Dialog_MSAT.h"
#include <QDialog>

class Mesh_PlatformClass;
class ViewerQt;
namespace easy3d
{
	class Model;
}

class DialogMSAT_View : public QDialog, public Ui::DialogMSAT
{
	Q_OBJECT

public:
	explicit DialogMSAT_View(Mesh_PlatformClass *window);
	~DialogMSAT_View();


protected:

	ViewerQt *viewer_;
	Mesh_PlatformClass *window_;
	easy3d::Model* copyorigin;
	easy3d::Model* copysecond;

private Q_SLOTS:
	void apply_MSAT();
	void setShowFeatureEdges();
	void setShowFeaturevertices();
	void apply_FeatureGraph();

private:
	void updateVectorFieldBuffer(easy3d::Model *model, const std::string &name);
	void updateVertexFieldBuffer(easy3d::Model *model, const std::string &name);



};

