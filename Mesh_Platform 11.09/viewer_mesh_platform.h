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
	class TrianglesDrawable;//�����λ�������
	class TextRenderer;//��Ⱦ
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

	// the actual samples receivedʵ���յ�����Ʒ
	int samples() const { return samples_; }

	// Scaling factor for high DPI devices��DPI�����ı�������
	double dpi_scaling() const { return dpi_scaling_; }

	const easy3d::vec4& backGroundColor() const { return background_color_; }
	void setBackgroundColor(const easy3d::vec4& c);//���ñ�����ɫ

	void addModel(easy3d::Model* model);
	void deleteModel(easy3d::Model* model);

	const std::vector<easy3d::Model*>& models() const { return models_; }
	easy3d::Model* currentModel() const;
	easy3d::Model* originModel() const;
	easy3d::Model* secondModel() const;
	easy3d::Model* thirdModel() const;
	easy3d::Model* fourthModel() const;
	easy3d::Model* resultModel() const;
	void setCurrentModel(easy3d::Model *m);
	void setShowOnly();

	// the camera
	easy3d::Camera* camera() const { return camera_; }

	// moves the camera so that the 'model' is centered on the screen.
	// if 'model' is NULL, it centers the entire scene (all models).
	//�ƶ������ʹ��ģ�͡�λ����Ļ�����ġ�
    //���'model'ΪNULL������������(����ģ��)���С�
	void fitScreen(const easy3d::Model* model = nullptr);

	// Returns the coordinates of the 3D point located at pixel (x,y) on screen.
	// x, y: screen point expressed in pixel units with an origin in the upper left corner.
	// found: indicates whether a point was found or not.
	//����3D������Ļ�ϵ�����(x,y)��
    // x, y:�����ص�λ��ʾ����Ļ�㣬ԭ�������Ͻǡ�
    // found:��ʾ�Ƿ��ҵ�һ����
	// NOTE: This method assumes that a GL context is available, and that its
	//		 content was drawn using the Camera (i.e. using its projection and modelview
	//		 matrices). This method hence cannot be used for offscreen Camera computations.
	//		 Use cameraCoordinatesOf() and worldCoordinatesOf() to perform similar
	//		 operations in that case.
	//       The precision of the z-Buffer highly depends on how the zNear() and zFar()
	//       values are fitted to your scene. Loose boundaries will result in imprecision
	//		 along the viewing direction.
	//�����������GL�������ǿ��õģ���������������ʹ��Camera���Ƶ�(��ʹ������ͶӰ��ģ����ͼ����)��
	//�˷�����˲���������Ļ���������㡣ʹ��cameraCoordinatesOf()��worldCoordinatesOf()��ִ�����ƵĲ�����
	//z-Buffer�ľ��ȸ߶�������zNear()��zFar()ֵ����ʺ���ĳ�����
	//��ɢ�ı߽罫�����ع۲췽�򲻾�ȷ��
	easy3d::vec3 pointUnderPixel(const QPoint& p, bool &found) const;

	// Save snapshot�������
	// file_name: the image file name
	bool saveSnapshot(const QString& file_name);

protected:

	/* Set up required OpenGL resources/state and then calls user-defined init().
	 * This method is called once before the first call to paintGL() or resizeGL().
	 * ���������OpenGL��Դ/״̬��Ȼ������û������init()��
     * �÷����ڵ�һ�ε���paintGL()��resizeGL()֮ǰ����һ�Ρ�
	 * Note:
	 *  - Overload init() instead of this method to modify specific OpenGL state;
	 * ����init()����������������޸��ض���OpenGL״̬;
	 *  - The framebuffer is not yet available at this stage.
	 * �ڴ˽׶Σ�framebuffer�������á�
	 */
	virtual void initializeGL() override;

	/* User-defined initialization method.�û�����ĳ�ʼ��������
	 * This method is called within initializeGL() and should be overloaded to
	 * initialize OpenGL flags/resources, e.g.,���������initializeGL()�е��ã�Ӧ����������ʼ��OpenGL��־/��Դ������:
	 *  - OpenGL state modification;OpenGL״̬�޸�;
	 *  - shader program/texture/VAOs creation;��ɫ����/����/VAOs����;
	 *  - camera initialization;�������ʼ��;
	 *  - previous viewer state restoration;��ǰ�Ĳ鿴��״̬�ָ�;
	 *  - ...
	 * All OpenGL specific initializations must be done in this method.����OpenGL�ض��ĳ�ʼ���������������������ɡ�
	 * OpenGL context is not yet available in your viewer constructor.OpenGL����������Ĳ鿴�����캯���л������á�
	 * NOTE:
	 *  - If you derive you own viewer from this class, don't forget to call
	 *    Viewer::init() at the beginning of your inherited function.
	 * ������������������Լ��Ĳ鿴������Ҫ�����ڼ̳к����Ŀ�ͷ����viewer::init()��
	 *  - Do not call updateGL() in this method (resulting in an infinite loop).
	 * ��Ҫ����������е���updateGL()(��������ѭ��)��
	 */
	//virtual void init();

	/* Sets up the OpenGL viewport, projection, etc. Gets called whenever the
	 * widget has been resized (and also when it is shown for the first time
	 * because all newly created widgets get a resize event automatically).
	 * If you overload this method, first call the inherited method in which
	 * the projection matrix is updated.
	 * ����OpenGL��ͼ��ͶӰ�ȡ�
	 * ��С����������Сʱ����(�Լ�������һ����ʾʱ���ã�
	 * ��Ϊ�����´�����С���������Զ���ȡһ��������С�¼�)��
	 * ������ش˷��������ȵ��ø���ͶӰ����ļ̳з�����
	 */
	virtual void resizeGL(int width, int height) override;

	/* Renders the OpenGL scene. Gets called whenever the widget needs to
	 * be updated. Internally, it calls the following methods in order:
	 * ��ȾOpenGL����.��С������Ҫ����ʱ���á����ڲ��������ε������·���:
	 *  - preDraw(): places the camera in the world coordinate system;���������������������ϵ��;
	 *  - draw(): main drawing method. Should be overloaded.����ͼ������Ӧ���ǳ��ء�
	 *  - postDraw(): display of visual hints (world axis, FPS...)��ʾ�Ӿ���ʾ(�����ᣬFPS��)
	 * Note: For normal rendering, i.e., drawing triggered by the
	 *       paintEvent(), the clearing of the color and depth buffers is
	 *       done by the widget before entering paintGL(). However, if you
	 *       want to reuse the paintGL() method for offscreen rendering,
	 *       you have to clear both buffers before calling paintGL().
	 * ������ͨ����Ⱦ������paintEvent()�����Ļ��ƣ��ڽ���paintGL()֮ǰ��С���������ɫ����Ȼ�������
	 * ���ǣ��������Ҫ����paintGL()������������Ļ����֣�������ڵ���paintGL()֮ǰ���������������
	 */
	virtual void paintGL() override;

	/* This function will be called before the main draw procedure.����������������ƹ���֮ǰ�����á�
	 */
	virtual void preDraw();

	/* The core method of the viewer, that draws the scene.���ڵĺ��ķ�������������
	 */
	virtual void draw();

	/* Called after draw() to draw viewer visual hints.��draw()֮����ã����ڻ��Ʋ鿴�����ӻ���ʾ��
	 * By default, it displays axis and visual hints if the respective flags are set.
	 * Ĭ������£���������˸��Եı�־��������ʾ����Ӿ���ʾ��
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
	//OpenGL��Դ(���磬��ɫ��������VAOs)�����ڴ���һ����Ч����Ⱦ������ʱ���١�
	//����������������OpenGL(ͨ��)��һ�������⣬
	//��ΪOpenGL�����Ŀ��ܲ�����(���磬�Ѿ�������)��ɼ��Ĳ��ǵ�ǰ�ġ�
	//���cleanup()������Ϊ��ȷ������һ����Ч�ĳ��������ġ��μ�init()��
    //ע��:��Ҫ�����ڼ̳к�����ĩβ����Viewer::cleanup()��
	virtual void cleanup();

public:
	virtual void mousePressEvent(QMouseEvent *) override;    // Mouse button press event handler��갴ť�����¼��������
	virtual void mouseMoveEvent(QMouseEvent *) override;
	virtual void mouseReleaseEvent(QMouseEvent *) override;  // Mouse button release event handler��갴ť�ͷ��¼��������
	virtual void mouseDoubleClickEvent(QMouseEvent *) override;
	virtual void wheelEvent(QWheelEvent *) override;         // Mouse scroll event handler�������¼��������
	virtual void keyPressEvent(QKeyEvent *) override;        // Keyboard press event handler.���̰����¼��������
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
	//ʵ���ϣ��ҿ��Դ�QOpenGLFunctions�̳в鿴��(���û�������ĳ�Ա����)��
	//������Ϊ��Ա������������ͷ�ļ���
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




