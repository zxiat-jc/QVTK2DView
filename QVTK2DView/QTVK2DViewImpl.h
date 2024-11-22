#pragma once

#include "QVTK2DView.h"

#include <vtkActor.h>
#include <vtkSmartPointer.h>

#include <pcl/common/common_headers.h>
#include <pcl/common/distances.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

class vtkProp3D;
class vtkOrientationMarkerWidget;
class ZRPControlInteractor;
class QTVK2DViewImpl : public QVTK2DView {
    Q_OBJECT

public:
    enum ShapeType {
        Oval,
        Triangle,
        Point,
        Line
    };
    Q_ENUM(ShapeType)
    QTVK2DViewImpl(QWidget* parent);
    ~QTVK2DViewImpl();

    /**
     * @brief 创建获取基本坐标系视图
     * @return vtkNew<vtkAxesActor>
     */
    vtkSmartPointer<vtkAxesActor> getBaseAxes() override;

    void addPoint(QString name, Eigen::Vector2d point, QColor color = { 255, 0, 0 }) override;

    void addLine(Eigen::Vector2d p1, Eigen::Vector2d p2, QColor color = { 0, 0, 0 }) override;

    void refresh() override;

    /**
     * @brief 添加等边三角
     * @param center 三角中心
     * @param color 颜色
     */
    void addTriangle(Eigen::Vector2d center, QColor color = { 0, 0, 0 }) override;

    /**
     * @brief 添加font
     * @param name 点名
     * @param position 点位置
     * @param color 颜色
     */
    void addFont(QString name, Eigen::Vector2d position, const QString& id, QColor color = { 255, 0, 0 }) override;

    /**
     * @brief 导出scene为GLTF格式
     * @param path 导出路径
     * @return
     */
    bool exportGLTF(QString path) override;

    /**
     * @brief 添加演员
     * @param actor
     */
    void addActor(vtkSmartPointer<vtkActor> actor) override;

    /*void addBall(QVector3D center, double radius, QColor color = { 0, 255, 0 });*/

    void addOval(Eigen::Vector2d center, double a, double b, double a_angle, QColor color = { 0, 0, 0 }) override;

    void amplifyOval(int sacle) override;

    /**
     * @brief 加载场景
     * @param filePath GLTF路径
     */
    void loadScene(QString filePath) override;

    void clear() override;

    void setCamera() override;

    vtkSmartPointer<vtkCamera> camera() const override;

private:
    vtkSmartPointer<ZRPControlInteractor> _interactor = nullptr;

    vtkSmartPointer<vtkOrientationMarkerWidget> _borderWidget = nullptr;

    vtkSmartPointer<vtkAxesActor> _baseAxes = nullptr;

    pcl::visualization::PCLVisualizer::Ptr _viewer = nullptr;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud = nullptr;

    vtkSmartPointer<vtkCamera> _camera = nullptr;

    /**
     * @brief 对象映射
     */
    QMap<ShapeType, QList<vtkProp3D*>> _vtkActors;

    QList<QString> _fontIds;

    int _fontId = 0;

    constexpr static const int LINE_WIDTH = 2;

    constexpr static const int CIRCLE_RADIUS = 2;

    constexpr static const int TRIANGLE_RADIUS = 20;

    constexpr static const double DOUBLESOLIDLINE_OFFSET = 0.5;

    constexpr static const char* POINT_CLOUD = "POINT_CLOUD";
};
