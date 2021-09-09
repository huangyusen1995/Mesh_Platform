#pragma once

#include <string>
#include <easy3d/core/types.h>
/*
#include <QOpenGLWidget>
#include <QElapsedTimer>
*/
#include <QtWidgets\QOpenGLWidget>
#include <QtCore\QElapsedTimer>

namespace easy3d {
	class Camera;
	class Model;
	class TrianglesDrawable;//三角形绘制能力
	class TextRenderer;//渲染
}

class QWidget;
class QOpenGLFunctions;

class ViewerQt : public QOpenGLWidget
{
	Q_OBJECT
public:
	explicit ViewerQt(QWidget* parent = nullptr);
	virtual ~ViewerQt() override;

	virtual std::string usage() const;

	// the actual samples received实际收到的样品
	int samples() const { return samples_; }

	// Scaling factor for high DPI devices高DPI器件的比例因子
	double dpi_scaling() const { return dpi_scaling_; }

	const easy3d::vec4& backGroundColor() const { return background_color_; }
	void setBackgroundColor(const easy3d::vec4& c);//设置背景颜色

	void addModel(easy3d::Model* model);
	void deleteModel(easy3d::Model* model);

	const std::vector<easy3d::Model*>& models() const { return models_; }
	easy3d::Model* currentModel() const;
	easy3d::Model* originModel() const;
	easy3d::Model* secondModel() const;
	easy3d::Model* thirdModel() const;
	easy3d::Model* resultModel() const;
	void setCurrentModel(easy3d::Model *m);
	void setShowOnly();

	// the camera
	easy3d::Camera* camera() const { return camera_; }

	// moves the camera so that the 'model' is centered on the screen.
	// if 'model' is NULL, it centers the entire scene (all models).
	//移动相机，使“模型”位于屏幕的中心。
    //如果'model'为NULL，则整个场景(所有模型)居中。
	void fitScreen(const easy3d::Model* model = nullptr);

	// Returns the coordinates of the 3D point located at pixel (x,y) on screen.
	// x, y: screen point expressed in pixel units with an origin in the upper left corner.
	// found: indicates whether a point was found or not.
	//返回3D点在屏幕上的坐标(x,y)。
    // x, y:以像素单位表示的屏幕点，原点在左上角。
    // found:表示是否找到一个点
	// NOTE: This method assumes that a GL context is available, and that its
	//		 content was drawn using the Camera (i.e. using its projection and modelview
	//		 matrices). This method hence cannot be used for offscreen Camera computations.
	//		 Use cameraCoordinatesOf() and worldCoordinatesOf() to perform similar
	//		 operations in that case.
	//       The precision of the z-Buffer highly depends on how the zNear() and zFar()
	//       values are fitted to your scene. Loose boundaries will result in imprecision
	//		 along the viewing direction.
	//这个方法假设GL上下文是可用的，并且它的内容是使用Camera绘制的(即使用它的投影和模型视图矩阵)。
	//此方法因此不能用于屏幕外的相机计算。使用cameraCoordinatesOf()和worldCoordinatesOf()来执行类似的操作。
	//z-Buffer的精度高度依赖于zNear()和zFar()值如何适合你的场景。
	//松散的边界将导致沿观察方向不精确。
	easy3d::vec3 pointUnderPixel(const QPoint& p, bool &found) const;

	// Save snapshot保存快照
	// file_name: the image file name
	bool saveSnapshot(const QString& file_name);

protected:

	/* Set up required OpenGL resources/state and then calls user-defined init().
	 * This method is called once before the first call to paintGL() or resizeGL().
	 * 设置所需的OpenGL资源/状态，然后调用用户定义的init()。
     * 该方法在第一次调用paintGL()或resizeGL()之前调用一次。
	 * Note:
	 *  - Overload init() instead of this method to modify specific OpenGL state;
	 * 重载init()而不是这个方法来修改特定的OpenGL状态;
	 *  - The framebuffer is not yet available at this stage.
	 * 在此阶段，framebuffer还不可用。
	 */
	virtual void initializeGL() override;

	/* User-defined initialization method.用户定义的初始化方法。
	 * This method is called within initializeGL() and should be overloaded to
	 * initialize OpenGL flags/resources, e.g.,这个方法在initializeGL()中调用，应该重载来初始化OpenGL标志/资源，例如:
	 *  - OpenGL state modification;OpenGL状态修改;
	 *  - shader program/texture/VAOs creation;着色程序/纹理/VAOs创建;
	 *  - camera initialization;摄像机初始化;
	 *  - previous viewer state restoration;以前的查看器状态恢复;
	 *  - ...
	 * All OpenGL specific initializations must be done in this method.所有OpenGL特定的初始化都必须在这个方法中完成。
	 * OpenGL context is not yet available in your viewer constructor.OpenGL上下文在你的查看器构造函数中还不可用。
	 * NOTE:
	 *  - If you derive you own viewer from this class, don't forget to call
	 *    Viewer::init() at the beginning of your inherited function.
	 * 如果你从这个类派生你自己的查看器，不要忘记在继承函数的开头调用viewer::init()。
	 *  - Do not call updateGL() in this method (resulting in an infinite loop).
	 * 不要在这个方法中调用updateGL()(导致无限循环)。
	 */
	//virtual void init();

	/* Sets up the OpenGL viewport, projection, etc. Gets called whenever the
	 * widget has been resized (and also when it is shown for the first time
	 * because all newly created widgets get a resize event automatically).
	 * If you overload this method, first call the inherited method in which
	 * the projection matrix is updated.
	 * 设置OpenGL视图，投影等。
	 * 在小部件调整大小时调用(以及在它第一次显示时调用，
	 * 因为所有新创建的小部件都会自动获取一个调整大小事件)。
	 * 如果重载此方法，首先调用更新投影矩阵的继承方法。
	 */
	virtual void resizeGL(int width, int height) override;

	/* Renders the OpenGL scene. Gets called whenever the widget needs to
	 * be updated. Internally, it calls the following methods in order:
	 * 渲染OpenGL场景.在小部件需要更新时调用。在内部，它依次调用以下方法:
	 *  - preDraw(): places the camera in the world coordinate system;将摄像机放置在世界坐标系中;
	 *  - draw(): main drawing method. Should be overloaded.主绘图方法。应该是超载。
	 *  - postDraw(): display of visual hints (world axis, FPS...)显示视觉提示(世界轴，FPS…)
	 * Note: For normal rendering, i.e., drawing triggered by the
	 *       paintEvent(), the clearing of the color and depth buffers is
	 *       done by the widget before entering paintGL(). However, if you
	 *       want to reuse the paintGL() method for offscreen rendering,
	 *       you have to clear both buffers before calling paintGL().
	 * 对于普通的渲染，即由paintEvent()触发的绘制，在进入paintGL()之前由小部件清除颜色和深度缓冲区。
	 * 但是，如果您想要重用paintGL()方法来进行屏幕外呈现，则必须在调用paintGL()之前清除两个缓冲区。
	 */
	virtual void paintGL() override;

	/* This function will be called before the main draw procedure.这个函数将在主绘制过程之前被调用。
	 */
	virtual void preDraw();

	/* The core method of the viewer, that draws the scene.观众的核心方法，画场景。
	 */
	virtual void draw();

	/* Called after draw() to draw viewer visual hints.在draw()之后调用，用于绘制查看器可视化提示。
	 * By default, it displays axis and visual hints if the respective flags are set.
	 * 默认情况下，如果设置了各自的标志，它将显示轴和视觉提示。
	 */
	virtual void postDraw();

	// OpenGL resources (e.g., shaders, textures, VAOs) must destroyed when
	// there exists a valid rendering context. It is (usually) a bad idea to
	// clean up OpenGL in a destructor because the OpenGL context may not exist
	// (e.g., destroyed already) or the visible one is not *current*. This
	// cleanup() function is to ensure you have a valid rendering context.
	// See also init().
	// NOTE: Don't forget to call Viewer::cleanup() at the end of your
	//		 inherited function.
	//OpenGL资源(例如，着色器，纹理，VAOs)必须在存在一个有效的渲染上下文时销毁。
	//在析构函数中清理OpenGL(通常)是一个坏主意，
	//因为OpenGL上下文可能不存在(例如，已经被销毁)或可见的不是当前的。
	//这个cleanup()函数是为了确保您有一个有效的呈现上下文。参见init()。
    //注意:不要忘记在继承函数的末尾调用Viewer::cleanup()。
	virtual void cleanup();

public:
	virtual void mousePressEvent(QMouseEvent *) override;    // Mouse button press event handler鼠标按钮按下事件处理程序
	virtual void mouseMoveEvent(QMouseEvent *) override;
	virtual void mouseReleaseEvent(QMouseEvent *) override;  // Mouse button release event handler鼠标按钮释放事件处理程序
	virtual void mouseDoubleClickEvent(QMouseEvent *) override;
	virtual void wheelEvent(QWheelEvent *) override;         // Mouse scroll event handler鼠标滚动事件处理程序
	virtual void keyPressEvent(QKeyEvent *) override;        // Keyboard press event handler.键盘按下事件处理程序。
	virtual void keyReleaseEvent(QKeyEvent *) override;      // Keyboard press event handler.
	virtual void timerEvent(QTimerEvent *) override;
	virtual void closeEvent(QCloseEvent *) override;

	friend class WidgetRendering;
protected:

	void drawCornerAxes();

signals:
	void currentModelChanged();

protected:
	// Actually I can inherit the viewer from QOpenGLFunctions (thus no such a member 
	// variable). Having it as a member can eliminate including the header file.
	//实际上，我可以从QOpenGLFunctions继承查看器(因此没有这样的成员变量)。
	//将它作为成员可以消除包括头文件。
	QOpenGLFunctions* func_;

	double  dpi_scaling_;
	int     samples_;

	QElapsedTimer timer_;
	easy3d::TextRenderer* texter_;

	easy3d::Camera*	camera_;
	easy3d::vec4	background_color_;

	Qt::MouseButton pressed_button_;
	QPoint  mouse_pressed_pos_;
	QPoint  mouse_previous_pos_;

	bool    show_pivot_point_;

	//----------------- viewer data -------------------

	// corner axes
	easy3d::TrianglesDrawable* drawable_axes_;

	std::vector<easy3d::Model*> models_;
	int model_idx_;
};




