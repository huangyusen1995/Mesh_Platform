
#ifndef WIDGET_RERNDERING_H
#define WIDGET_RERNDERING_H

#include <QtWidgets/QWidget>
#include "ui_Widget_Rendering.h"

//class Mesh_PlatformClass;
class ViewerQt;
class Mesh_PlatformClass;

namespace Ui {
	class WidgetRendering;
}


class WidgetRendering : public QWidget
{
	Q_OBJECT

public:
	explicit WidgetRendering(QWidget *parent);
	//explicit WidgetRendering(Mesh_PlatformClass *window);

	~WidgetRendering() override;

	// update the panel to be consistent with the drawable's rendering parameters
	//void updatePanel() override;
protected:

	ViewerQt *viewer_;
	Mesh_PlatformClass *window_;


public slots:
	
	void setPointVisible(bool b);
	void setEdgeVisible(bool b);
	//void setEdgeWidth();
	void setTriangleVisible(bool b);
	void setFaceNormalVisible(bool b);
	//void setinputModelVisible();
	void original_model();
private:
	Ui::WidgetRendering *ui;
	//void updateNormalFieldBuffer(easy3d::Model *model, const std::string &name);


};

#endif // WIDGET_DRAWABLE_LINES_H
