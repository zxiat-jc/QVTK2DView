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
        Circle,
        Oval,
        Triangle,
        Point,
        Line
    };
    Q_ENUM(ShapeType)

    struct Font {
        QString text = "";
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    QTVK2DViewImpl(QWidget* parent);
    ~QTVK2DViewImpl();

    /**
     * @brief 创建获取基本坐标系视图
     * @return vtkNew<vtkAxesActor>
     */
    vtkSmartPointer<vtkAxesActor> getBaseAxes();

    /**
     * @brief 添加plc点
     * @param name
     * @param point
     * @param color
     */
    void addPoint(QString name, Eigen::Vector2d point, QColor color = { 0, 0, 0 }) override;

    /**
     * @brief 添加vtk点
     * @param name
     * @param xy
     * @param color
     */
    void addVtkPoint(QString name, Eigen::Vector2d xy, QColor color = { 0, 0, 0 }) override;

    /**
     * @brief 添加线
     * @param p1
     * @param p2
     * @param color
     */
    void addLine(Eigen::Vector2d p1, Eigen::Vector2d p2, QColor color = { 0, 0, 0 }) override;

    /**
     * @brief 添加双实线
     * @param p1
     * @param p2
     * @param color
     */
    void addDoubleLine(Eigen::Vector2d p1, Eigen::Vector2d p2, QColor color = { 0, 0, 0 }) override;

    /**
     * @brief 刷新
     */
    void refresh() override;

    /**
     * @brief 添加等边三角
     * @param center 三角中心
     * @param color 颜色
     */
    void addTriangle(QString name, Eigen::Vector2d center, QColor color = { 0, 0, 0 }) override;

    /**
     * @brief 添加font
     * @param name 点名
     * @param position 点位置
     * @param color 颜色
     */
    void addFont(QString name, Eigen::Vector2d position, const QString& id, QColor color = { 0, 0, 255 }) override;

    /**
     * @brief 导出scene为GLTF格式
     * @param path 导出路径
     * @return
     */
    bool exportGLTF(QString path) override;

    /**
     * @brief 导出scene为DXF格式
     * @param path 导出路径
     * @return
     */
    bool exportDXF(QString path) override;

    /**
     * @brief 添加椭圆
     * @param center 中心
     * @param a 长轴
     * @param b 短轴
     * @param a_angle 倾斜角
     * @param color 颜色
     */
    void addOval(Eigen::Vector2d center, double a, double b, double a_angle = 0, QColor color = { 0, 0, 0 }) override;

    /**
     * @brief
     * @param center
     * @param radius
     * @param color
     */
    void addCircle(Eigen::Vector2d center, double radius, QColor color = { 0, 0, 0 }) override;

    /**
     * @brief 缩放椭圆
     * @param sacle
     */
    void amplifyOval(int sacle) override;

    /**
     * @brief 清除画布
     */
    void clear() override;

    /**
     * @brief 根据点云范围设置相机视角
     */
    void setCameraBaseOnCloud() override;

    /**
     * @brief 根据vtk点设置相机视角
     */
    void setCameraBaseOnVtkPoint() override;

    /**
     * @brief 过滤器
     */
    bool eventFilter(QObject* obj, QEvent* event);

    /**
     * @brief 更新三角形大小
     */
    void updateTriangle() override;

    /**
     * @brief 窗口高度与世界坐标系中的像素高度的转换
     * @return
     */
    double windowHeight2Pixel(double height);

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

    QMap<QString, Font> _fonts;

    int _fontSize = 1000;

    std::optional<Eigen::Vector2d> _center = std::nullopt;

    constexpr static const int FONT_SCALE_STEP = 5;

    constexpr static const int LINE_WIDTH = 0.5;

    constexpr static const int CIRCLE_RADIUS = 2;

    constexpr static const int TRIANGLE_RADIUS = 20;

    constexpr static const double DOUBLESOLIDLINE_OFFSET = 0.5;

    constexpr static const char* POINT_CLOUD = "POINT_CLOUD";
};
