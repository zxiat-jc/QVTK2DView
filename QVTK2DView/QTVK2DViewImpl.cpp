#include "QTVK2DViewImpl.h"

#include <Eigen/Core>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vtkAppendPolyData.h>
#include <vtkAxesActor.h>
#include <vtkCamera.h>
#include <vtkCaptionActor2D.h>
#include <vtkCollectionIterator.h>
#include <vtkDataSetMapper.h>
#include <vtkDiskSource.h>
#include <vtkExtractEdges.h>
#include <vtkFreeTypeStringToImage.h>
#include <vtkGLTFExporter.h>
#include <vtkGLTFImporter.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkImageData.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkLegendScaleActor.h>
#include <vtkLine.h>
#include <vtkLineSource.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkOutputWindow.h>
#include <vtkParametricEllipsoid.h>
#include <vtkParametricFunctionSource.h>
#include <vtkPlaneSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkProperty2d.h>
#include <vtkRegularPolygonSource.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkSmartPointer.h>
#include <vtkSmoothPolyDataFilter.h>
#include <vtkSphereSource.h>
#include <vtkTextProperty.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

#include <QTimer>
#include <QVector3D>
class ZRPControlInteractor : public vtkInteractorStyleTrackballCamera {
private:
    bool _zoom = true;
    bool _rotate = false;
    bool _pan = true;

public:
    /**
     * @brief 缩放
     * @param zoom 是否启用
     */
    void setZoom(bool zoom) { _zoom = zoom; }

    /**
     * @brief 旋转
     * @param rotate 是否启用
     */
    void setRotate(bool rotate) { _rotate = rotate; }

    /**
     * @brief 平移
     * @param pan 是否启用
     */
    void setPan(bool pan) { _pan = pan; }

    static ZRPControlInteractor* New();

    ZRPControlInteractor()
    {
    }

    void StartZoom() override
    {
        if (_zoom) {
            Superclass::StartZoom();
        }
    }
    void StartRotate() override
    {
        if (_rotate) {
            Superclass::StartRotate();
        }
    }
    void StartPan() override
    {
        if (_pan) {
            Superclass::StartPan();
        }
    }

public:
    vtkTypeMacro(ZRPControlInteractor,
        vtkInteractorStyle)
};

vtkStandardNewMacro(ZRPControlInteractor);

QTVK2DViewImpl::QTVK2DViewImpl(QWidget* parent)
{
    vtkOutputWindow::SetGlobalWarningDisplay(0);

    auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    auto renderer = vtkSmartPointer<vtkRenderer>::New();

    _interactor = vtkSmartPointer<ZRPControlInteractor>::New();
    _interactor->SetDefaultRenderer(renderer);

    renderWindow->AddRenderer(renderer);
    this->setRenderWindow(renderWindow);
    renderWindow->GetInteractor()->SetInteractorStyle(_interactor);

    this->setEnableHiDPI(true);
    // 开启抗锯齿
    renderWindow->SetMultiSamples(4);
    renderer->SetUseFXAA(true);
    // 设置页面底部颜色值
    renderer->SetBackground(1.0, 1.0, 1.0);
    // 设置页面顶部颜色值
    renderer->SetBackground2(0.529, 0.8078, 0.92157);
    // 开启渐变色背景设置
    renderer->SetGradientBackground(true);
    renderer->SetUseDepthPeeling(1);
    renderer->SetOcclusionRatio(0.4);
    renderer->UseDepthPeelingOn();

    _viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "", false));
    _cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    this->_viewer->addPointCloud(_cloud, POINT_CLOUD);
    // 设置POINT_CLOUD点大小
    this->_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, POINT_CLOUD);

    // 添加中心坐标轴
    renderer->AddActor(_baseAxes = getBaseAxes());
    // 激活中心坐标
    this->_borderWidget = vtkOrientationMarkerWidget::New();
    vtkSmartPointer<vtkAxesActor> widgetAxesActor = vtkSmartPointer<vtkAxesActor>::New();
    widgetAxesActor->SetXAxisLabelText("X");
    widgetAxesActor->SetYAxisLabelText("Y");
    widgetAxesActor->SetZAxisLabelText("");
    widgetAxesActor->SetTotalLength(1, 1, 0);
    widgetAxesActor->SetPosition(0, 0, 0);
    widgetAxesActor->SetShaftType(0);
    widgetAxesActor->SetCylinderRadius(0.02);
    // 激活同步坐标小窗,
    this->_borderWidget->SetOrientationMarker(widgetAxesActor);
    this->_borderWidget->SetInteractor(this->interactor());
    this->_borderWidget->SetEnabled(true);
    this->_borderWidget->InteractiveOn();
    _camera = renderer->GetActiveCamera();
    // 设置位置(1000,1000,1000)
    _camera->SetPosition(0, 0, 1000);
    // 可视角度45度
    _camera->SetViewAngle(45);
}

QTVK2DViewImpl::~QTVK2DViewImpl()
{
}

vtkSmartPointer<vtkAxesActor> QTVK2DViewImpl::getBaseAxes()
{
    vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
    axes->SetZAxisLabelText("");
    axes->SetTotalLength(1000, 1000, 0);
    axes->SetPosition(0, 0, 0);
    axes->SetShaftType(0);
    axes->SetConeResolution(100);
    axes->SetCylinderResolution(100);
    axes->SetConeRadius(0.1);
    axes->SetCylinderRadius(0.01);
    // 隐藏标签label
    axes->AxisLabelsOff();
    axes->GetXAxisCaptionActor2D()->GetProperty()->SetOpacity(0.7);
    axes->GetYAxisCaptionActor2D()->GetProperty()->SetOpacity(0.7);
    axes->GetZAxisCaptionActor2D()->GetProperty()->SetOpacity(0.7);
    return axes;
}

void QTVK2DViewImpl::addPoint(QString name, Eigen::Vector2d point, QColor color)
{
    _cloud->push_back(pcl::PointXYZRGB(point.x(), point.y(), 0, color.red(), color.green(), color.blue()));
    this->addFont(name, point, name);
}

void QTVK2DViewImpl::addLine(Eigen::Vector2d p1, Eigen::Vector2d p2, QColor color)
{
    vtkSmartPointer<vtkLineSource> lineSource = vtkSmartPointer<vtkLineSource>::New();
    lineSource->SetPoint1(p1.x(), p1.y(), 0);
    lineSource->SetPoint2(p2.x(), p2.y(), 0);
    lineSource->Update();

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(lineSource->GetOutputPort());
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    // 颜色
    actor->GetProperty()->SetColor(color.red(), color.green(), color.blue());
    // 透明度
    actor->GetProperty()->SetOpacity(1);
    // 线宽
    actor->GetProperty()->SetLineWidth(LINE_WIDTH);
    // 设置Actor的属性以填充颜色
    actor->GetProperty()->SetRepresentationToSurface();
    this->renderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(actor);
    this->_vtkActors[ShapeType::Line].push_back(actor);
}

void QTVK2DViewImpl::refresh()
{
    QTimer::singleShot(100, [this]() {
        this->_viewer->updatePointCloud(_cloud, POINT_CLOUD);
        this->renderWindow()->Modified();
        this->renderWindow()->Render();
    });
}

void QTVK2DViewImpl::addTriangle(Eigen::Vector2d center, QColor color)
{
    // 创建等边三角形
    vtkSmartPointer<vtkRegularPolygonSource> triangleSource = vtkSmartPointer<vtkRegularPolygonSource>::New();
    triangleSource->SetNumberOfSides(3);
    triangleSource->SetRadius(200);
    triangleSource->SetCenter(center.x(), center.y(), 0);
    // 创建Mapper和Actor
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(triangleSource->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    // 设置三角形属性
    actor->GetProperty()->SetOpacity(0.2); // 设置透明度
    actor->GetProperty()->SetColor(color.red(), color.green(), color.blue());
    actor->GetProperty()->SetLineWidth(2);
    actor->GetProperty()->SetRepresentationToWireframe(); // 设置为线框显示
    this->renderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(actor);
    this->_vtkActors[ShapeType::Triangle].push_back(actor);
    this->addFont("triangle", center, "triangle");
    this->renderWindow()->Render();
}

void QTVK2DViewImpl::addFont(QString name, Eigen::Vector2d position, const QString& id, QColor color)
{
    struct {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    } p { position.x(), position.y(), 0 };
    this->_viewer->addText3D(name.toUtf8().data(), p, 500, color.redF(), color.greenF(), color.blueF(), id.toStdString());
    this->_fontIds.push_back(id);
}

bool QTVK2DViewImpl::exportGLTF(QString path)
{
    vtkNew<vtkGLTFExporter> writer;
    writer->SetFileName(path.toStdString().c_str());
    writer->InlineDataOn();
    writer->SetRenderWindow(this->renderWindow());
    writer->Write();
    return true;
}

void QTVK2DViewImpl::addActor(vtkSmartPointer<vtkActor> actor)
{
    if (actor == nullptr) {
        return;
    }
    // this->_actors.append(actor);
    this->renderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(actor);
    this->renderWindow()->Render();
}

void QTVK2DViewImpl::addOval(Eigen::Vector2d center, double a, double b, double a_angle, QColor color)
{
    // 创建椭圆源
    vtkSmartPointer<vtkRegularPolygonSource> ellipseSource = vtkSmartPointer<vtkRegularPolygonSource>::New();
    ellipseSource->SetNumberOfSides(100); // 设置多边形的边数
    ellipseSource->SetRadius(a); // 设置半径
    ellipseSource->SetCenter(0, 0, 0);

    // 只保留边框
    vtkSmartPointer<vtkExtractEdges> extractEdges = vtkSmartPointer<vtkExtractEdges>::New();
    extractEdges->SetInputConnection(ellipseSource->GetOutputPort());
    extractEdges->Update();

    // 创建变换对象
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->Scale(b / a, 1, 0);

    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformFilter->SetInputConnection(extractEdges->GetOutputPort());
    transformFilter->SetTransform(transform);
    transformFilter->Update();

    // 创建Mapper和Actor
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(transformFilter->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->RotateZ(180 - a_angle);

    vtkSmartPointer<vtkTransform> transform2 = vtkSmartPointer<vtkTransform>::New();
    transform2->Translate(center.x(), center.y(), 0);
    actor->SetUserTransform(transform2);

    // 设置Actor属性
    vtkSmartPointer<vtkProperty> property = actor->GetProperty();
    property->SetRepresentationToWireframe();
    property->SetColor(0.0, 0.0, 0.0);

    this->renderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(actor);
    this->_vtkActors[ShapeType::Oval].push_back(actor);
}

void QTVK2DViewImpl::amplifyOval(int emplifySacle)
{
    for (auto&& actor : this->_vtkActors[ShapeType::Oval]) {
        double* scale = actor->GetScale();
        actor->SetScale(emplifySacle, emplifySacle, 1);
    }
    this->refresh();
}

void QTVK2DViewImpl::loadScene(QString filePath)
{
    if (!filePath.endsWith(".gltf")) {
        qDebug() << "not obj";
        return;
    }
    vtkNew<vtkGLTFImporter> importer;
    importer->SetFileName(filePath.toStdString().c_str());
    importer->SetRenderWindow(this->renderWindow());
    importer->Update();
}

void QTVK2DViewImpl::clear()
{
    _cloud->clear();
    for (auto&& type : this->_vtkActors) {
        for (auto&& actor : type) {
            this->renderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(actor);
        }
    }
    for (auto&& id : _fontIds) {
        this->_viewer->removeShape(id.toStdString());
    }
    this->_fontIds.clear();
    this->_vtkActors.clear();
    this->refresh();
}

void QTVK2DViewImpl::setCamera()
{
    double xMin = this->_cloud->points[0].x;
    double xMax = this->_cloud->points[0].x;
    double yMin = this->_cloud->points[0].y;
    double yMax = this->_cloud->points[0].y;
    for (auto&& p : this->_cloud->points) {
        double currentX = p.x;
        double currentY = p.y;
        if (currentX < xMin) {
            xMin = currentX;
        } else if (currentX > xMax) {
            xMax = currentX;
        }

        if (currentY < yMin) {
            yMin = currentY;
        } else if (currentY > yMax) {
            yMax = currentY;
        }
    }
    double xAxisMaxLength = xMax - xMin;
    double yAxisMaxLength = yMax - yMin;
    double cameraZ = xAxisMaxLength > yAxisMaxLength ? xAxisMaxLength : yAxisMaxLength;

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*_cloud, centroid);

    // 创建一个Kd树对象
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(this->_cloud);
    // 最近邻搜索
    int K = 1; // 找到最近的一个点
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    kdtree.nearestKSearch({ centroid.x(), centroid.y(), 0 }, K, pointIdxNKNSearch, pointNKNSquaredDistance);

    this->_camera->SetPosition(_cloud->points[pointIdxNKNSearch[0]].x, _cloud->points[pointIdxNKNSearch[0]].y, cameraZ);
    this->_camera->SetFocalPoint(_cloud->points[pointIdxNKNSearch[0]].x, _cloud->points[pointIdxNKNSearch[0]].y, _cloud->points[pointIdxNKNSearch[0]].z);
}

vtkSmartPointer<vtkCamera> QTVK2DViewImpl::camera() const
{
    return this->_camera;
}
