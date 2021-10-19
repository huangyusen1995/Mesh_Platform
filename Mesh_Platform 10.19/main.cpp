#include "Mesh_Platform.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	Mesh_PlatformClass w;
	w.show();
	return a.exec();
}
