#include <QApplication>
#include <QtCore/QCoreApplication>

#include "PluginInterface.h"
#include "QPluginManager.h"
#include "QVTK2DView.h"
#include "QtUtils.h"
#include <Eigen/Core>
int main(int argc, char* argv[])
{
    QApplication a(argc, argv);

    QPluginManager::Instance().findLoadPlugins(QApplication::applicationDirPath() + "/plugins");
    qInfo() << "加载插件:" << QPluginManager::Instance().pluginNames();
    QString error;
    QPluginManager::Instance().initializes(a.arguments(), error);
    QPluginManager::Instance().extensionsInitialized();
    QPluginManager::Instance().delayedInitialize();

    if (VTK_VIEWER != nullptr) {
        VTK_VIEWER->deleteLater();
    }
    RegisterPropertyPtr(GetQClassName(QVTK2DView), QVTK2DView::New());
    VTK_VIEWER->addLine(Eigen::Vector2d(0, 0), Eigen::Vector2d(100, 100), QColor(255, 0, 0));
    VTK_VIEWER->addPoint("hello world", Eigen::Vector2d(100, 100), QColor(0, 0, 255));

    VTK_VIEWER->refresh();
    VTK_VIEWER->exportDXF("test.dxf");
    VTK_VIEWER->show();
    return a.exec();
}
