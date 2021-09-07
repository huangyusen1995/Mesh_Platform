
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
public slots: //��

	// file
	bool onOpen();//���ļ�
	bool onSave();//�����ļ�
	void onOpenRecentFile();//��������ļ�
	void onClearRecentFiles();//�ر�������ļ�
	void onCurrentModelChanged();//��ǰģ�͸ı�

	void updateUi();


	// view
	void saveSnapshot();//�����ͼ
	void setBackgroundColor();//���ñ�����ɫ

	// topology
	void reportTopologyStatistics();//��������ͳ��(���Topology�е�Statistic)

	// Algorithm
	void FeaturePreservingMeshSimplification();
	//error
	//void mean_error(SurfaceMesh *original_mesh, SurfaceMesh *result_mesh);
	// about
	void onAbout();//���ڣ����Help�е�About Mesh_Platform)
	
protected:
	void dragEnterEvent(QDragEnterEvent *) override;//������ק
	void dropEvent(QDropEvent *) override;//������ק�ͷ�
	void closeEvent(QCloseEvent *) override;//���ڸʱ�


private:

	// open a file (given the file name) and add the model to the viewer
	// for visualization. It will also create the default drawables for
	// visualizing the model.��һ���ļ�(�����ļ���)����ģ����ӵ��鿴�����Խ��п��ӻ���
	//�������������ڿ��ӻ�ģ�͵�Ĭ��drawables��
	easy3d::Model* open(const std::string& file_name);
public:
	friend class WidgetRendering;
	
private:
	void createActions();

	void createActionsForFileMenu();
	void createActionsForViewMenu();
	void createActionsForTopologyMenu();
	void createActionsForAlgorithmMenu();
	//void createActionsForErrorMenu();

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