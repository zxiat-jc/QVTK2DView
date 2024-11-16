#pragma once

#include "qvtk2dview_global.h"

#include <QVTKOpenGLNativeWidget.h>
#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkCamera.h>

#include "QtUtils.h"

#ifndef VTK_VIEWER
#define VTK_VIEWER []() { return GetAppPropertyPtr(GetQClassName(QVTK2DView), QVTK2DView); }()
#endif

class vtkOrientationMarkerWidget;
class QVTK2DVIEW_EXPORT QVTK2DView : public QVTKOpenGLNativeWidget {
    Q_OBJECT

public:
    QVTK2DView(QWidget* parent = nullptr);
    static QVTK2DView* New(QWidget* parent = nullptr);

    virtual ~QVTK2DView() = 0;

    virtual vtkSmartPointer<vtkAxesActor> getBaseAxes() = 0;

    virtual void addPoint(QString name, Eigen::Vector2d point, QColor color = { 0, 0, 0 }) = 0;
    virtual void addLine(Eigen::Vector2d p1, Eigen::Vector2d p2, QColor color = { 0, 0, 0 }) = 0;

    /**
     * @brief 添加等边三角
     * @param center 三角中心
     * @param color 颜色
     */
    virtual void addTriangle(Eigen::Vector2d center, QColor color = { 0, 0, 0 }) = 0;

    /**
     * @brief 添加font
     * @param name 点名
     * @param position 点位置
     * @param color 颜色
     */
    virtual void addFont(QString name, Eigen::Vector2d position, const QString& id, QColor color = { 0, 0, 0 }) = 0;

    /**
     * @brief 导出scene为GLTF格式
     * @param path 导出路径
     * @return
     */
    virtual bool exportGLTF(QString path) = 0;

    /**
     * @brief 添加演员
     * @param actor
     */
    virtual void addActor(vtkSmartPointer<vtkActor> actor) = 0;

    /* virtual void addBall(QVector3D center, double radius, QColor color = { 0, 0, 0 }) = 0;*/

    virtual void addOval(Eigen::Vector2d center, double a, double b, double a_angle, QColor color = { 0, 0, 0 }) = 0;

    virtual void amplifyOval(int sacle) = 0;
    /**
     * @brief 加载场景
     * @param filePath GLTF路径
     */
    virtual void loadScene(QString filePath) = 0;

    virtual void clear() = 0;

    virtual void refresh() = 0;

    virtual void setCamera() = 0;

    virtual vtkSmartPointer<vtkCamera> camera() const = 0;
};
