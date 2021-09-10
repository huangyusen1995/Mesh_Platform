
#ifndef MESH_PLATFORM_H
#define MESH_PLATFORM_H
#include <QtWidgets/QMainWindow>
#include "ui_Mesh_Platform.h"
#include <string>

namespace Ui {
	class Mesh_PlatformClass;
}

namespace easy3d {
	class Model;
}

class ViewerQt;
class WidgetRendering;


class Mesh_PlatformClass : public QMainWindow
{
	Q_OBJECT

public:
	//Mesh_Platform(QWidget *parent = Q_NULLPTR);
	explicit Mesh_PlatformClass(QWidget *parent = nullptr);
	~Mesh_PlatformClass() override;

	ViewerQt* viewer() { return viewer_; }
public slots: //槽

	// file
	bool onOpen();//打开文件
	bool onSave();//保存文件
	void onOpenRecentFile();//打开最近的文件
	void onClearRecentFiles();//关闭最近的文件
	void onCurrentModelChanged();//当前模型改变

	void updateUi();


	// view
	void saveSnapshot();//保存截图
	void setBackgroundColor();//设置背景颜色

	// topology
	void reportTopologyStatistics();//报告拓扑统计(点击Topology中的Statistic)

	// Algorithm
	void FeaturePreservingMeshSimplification();
	
	//error
	void runColorMap();
	// about
	void onAbout();//关于（点击Help中的About Mesh_Platform)
	
protected:
	void dragEnterEvent(QDragEnterEvent *) override;//窗口拖拽
	void dropEvent(QDropEvent *) override;//窗口拖拽释放
	void closeEvent(QCloseEvent *) override;//窗口甘比


private:

	// open a file (given the file name) and add the model to the viewer
	// for visualization. It will also create the default drawables for
	// visualizing the model.打开一个文件(给定文件名)并将模型添加到查看器中以进行可视化。
	//它还将创建用于可视化模型的默认drawables。
	easy3d::Model* open(const std::string& file_name);
public:
	friend class WidgetRendering;
	
private:
	void createActions();

	void createActionsForFileMenu();
	void createActionsForViewMenu();
	void createActionsForTopologyMenu();
	void createActionsForAlgorithmMenu();
	void createActionsForErrorMenu();

	bool okToContinue();
	void readSettings();
	void writeSettings();
	void updateWindowTitle();

	void setCurrentFile(const QString &fileName);

	void updateRecentFileActions();


	QString strippedName(const QString &fullFileName);
	
private:
	ViewerQt*   viewer_;
	//Model* opened_model;
	QStringList recentFiles_;
	QString		curDataDirectory_;

	enum { MaxRecentFiles = 5 };
	QAction *actionsRecentFile[MaxRecentFiles],
		*actionSeparator;

	WidgetRendering*  WidgetRendering_;
private:
	Ui::Mesh_PlatformClass *ui;
};

#endif 