
#ifndef WIDGET_RERNDERING_H
#define WIDGET_RERNDERING_H

#include <QtWidgets/QWidget>
#include <easy3d/renderer/drawable.h>
#include "ui_Widget_Rendering.h"

class ViewerQt;
class Mesh_PlatformClass;
namespace easy3d {
	class Texture;
}


namespace Ui {
	class WidgetRendering;
}


class WidgetRendering : public QWidget
{
	Q_OBJECT

public:
	explicit WidgetRendering(QWidget *parent);

	~WidgetRendering() override;

protected:

	ViewerQt *viewer_;
	Mesh_PlatformClass *window_;
	QString         scalar_prefix_;

	// the extended rendering state
	struct StateExts {
		StateExts() : scalar_style(0), discrete_color(false), num_stripes(16), vector_field("disabled"),
			vector_field_scale(1.0), highlight_range(-1, -1) {}
		int scalar_style;
		bool discrete_color;
		int num_stripes;
		QString vector_field;
		double vector_field_scale;
		// the face range for highlighting polygon faces (not the triangle range)
		std::pair<int, int> highlight_range;
	};

	struct ColorMaps {
		ColorMaps(const std::string& f, const std::string& n) : file(f), name(n), texture(nullptr) {}
		std::string file;
		std::string name;
		easy3d::Texture* texture;
	};

	//easy3d::Texture* colormap_Texture(int, bool, int) const;
	easy3d::Drawable* drawableTriangles();
	static std::vector<ColorMaps> colormaps;

public slots:
	
	void setPointVisible(bool b);
	void setEdgeVisible(bool b);
	//void setEdgeWidth();
	void setTriangleVisible(bool b);
	void setFaceNormalVisible(bool b);
	void original_model();
	void result_model();
	void clear_model();
	void set_ScalarFieldStyle(const QString &);
	void set_ColorScheme(const QString &);
	void setScalarFieldClamp(bool);
	void setScalarField_ClampLower(double);
	void setScalarField_ClampUpper(double);


private:
	Ui::WidgetRendering *ui;
	
private:

	std::unordered_map<easy3d::Model*, std::string> active_drawable_;
	std::unordered_map<easy3d::Drawable *, StateExts> states_;
	//std::vector<QString> colorSchemes(const easy3d::Model *model);

	//std::string current_texture;
	//std::string textureFile;
	//Texture *texture;
	
};

#endif 
