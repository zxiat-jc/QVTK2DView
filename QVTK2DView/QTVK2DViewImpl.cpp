#include "QTVK2DViewImpl.h"

#include <QKeyEvent>
#include <QTimer>
#include <QVBoxLayout>

#include <QVTKOpenGLNativeWidget.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vtkActor.h>
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
#include <vtkNamedColors.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkOutputWindow.h>
#include <vtkParametricEllipsoid.h>
#include <vtkParametricFunctionSource.h>
#include <vtkPlaneSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataMapper2D.h>
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
#include <vtkTriangle.h>

#include "libdxfrw.h"

#include "dx_iface.h"

#include "QtUtils.h"

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
    // 垂直布局
    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);
    this->setLayout(layout);
    this->_view = new QVTKOpenGLNativeWidget(this);
    layout->addWidget(this->_view);

    vtkOutputWindow::SetGlobalWarningDisplay(0);

    auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    auto renderer = vtkSmartPointer<vtkRenderer>::New();

    _interactor = vtkSmartPointer<ZRPControlInteractor>::New();
    _interactor->SetDefaultRenderer(renderer);

    renderWindow->AddRenderer(renderer);
    this->_view->setRenderWindow(renderWindow);
    renderWindow->GetInteractor()->SetInteractorStyle(_interactor);

    this->_view->setEnableHiDPI(true);
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
    this->_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, POINT_CLOUD);

    // 事件过滤器
    this->installEventFilter(this);
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
    this->_borderWidget->SetInteractor(this->_view->interactor());
    this->_borderWidget->SetEnabled(true);
    this->_borderWidget->InteractiveOn();
    _camera = renderer->GetActiveCamera();
    _camera->SetViewUp(0, 1, 0);

    // 设置位置(0,0,1000)
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

void QTVK2DViewImpl::addVtkPoint(QString name, Eigen::Vector2d xy, QColor color)
{
    vtkSmartPointer<vtkNamedColors> colors = vtkSmartPointer<vtkNamedColors>::New();
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    const float p[3] = { static_cast<float>(xy.x()), static_cast<float>(xy.y()), 0 };
    vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
    vtkIdType pid[1];
    pid[0] = points->InsertNextPoint(p);
    vertices->InsertNextCell(1, pid);

    vtkSmartPointer<vtkPolyData> point = vtkSmartPointer<vtkPolyData>::New();
    point->SetPoints(points);
    point->SetVerts(vertices);

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(point);

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(color.red(), color.green(), color.blue());
    actor->GetProperty()->SetPointSize(10);
    this->_view->renderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(actor);
    _vtkActors[ShapeType::Point].push_back(actor);
    this->addFont(name, xy, name);
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
    this->_view->renderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(actor);
    this->_vtkActors[ShapeType::Line].push_back(actor);
}

void QTVK2DViewImpl::addDoubleLine(Eigen::Vector2d p1, Eigen::Vector2d p2, QColor color)
{
    if (!this->_center.has_value()) {
        qDebug() << "中心点未设置";
        return;
    }
    int* winSize = this->_view->renderWindow()->GetSize();
    if (winSize[0] <= 0 || winSize[1] <= 0)
        return;
    int windowHeight = winSize[1];
    double length = qMax(this->_center.value().x(), this->_center.value().y()) * 2;
    double offset = length * 0.0000004;

    Eigen::Vector2d p1Up(p1.x(), p1.y() + offset);
    Eigen::Vector2d p2Up(p2.x(), p2.y() + offset);
    this->addLine(p1Up, p2Up, color);
    Eigen::Vector2d p1Down(p1.x(), p1.y() - offset);
    Eigen::Vector2d p2Down(p2.x(), p2.y() - offset);
    this->addLine(p1Down, p2Down, color);
}

void QTVK2DViewImpl::refresh()
{
    QTimer::singleShot(100, [this]() {
        this->_viewer->updatePointCloud(_cloud, POINT_CLOUD);
        this->_view->renderWindow()->Modified();
        this->_view->renderWindow()->Render();
    });
}

void QTVK2DViewImpl::addTriangle(QString name, Eigen::Vector2d center, QColor color)
{
    // 创建等边三角形
    vtkSmartPointer<vtkRegularPolygonSource> triangleSource = vtkSmartPointer<vtkRegularPolygonSource>::New();
    triangleSource->SetNumberOfSides(3);
    triangleSource->SetRadius(5);
    triangleSource->SetCenter(center.x(), center.y(), 0);
    // 创建Mapper和Actor
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(triangleSource->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    // 设置三角形属性
    actor->GetProperty()->SetOpacity(1); // 设置透明度
    actor->GetProperty()->SetColor(color.red(), color.green(), color.blue());
    actor->GetProperty()->SetLineWidth(2);
    actor->GetProperty()->SetRepresentationToSurface(); // 面填充
    // actor->GetProperty()->SetRepresentationToWireframe(); // 设置为线框显示
    this->_view->renderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(actor);
    this->_vtkActors[ShapeType::Triangle].push_back(actor);
    this->addFont(name, center, name);
}

void QTVK2DViewImpl::addFont(QString name, Eigen::Vector2d position, const QString& id, QColor color)
{
    struct {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    } p { position.x(), position.y(), 0 };
    this->_viewer->removeText3D(name.toUtf8().data());
    this->_viewer->addText3D(name.toUtf8().data(), p, this->_fontSize, color.redF(), color.greenF(), color.blueF(), id.toStdString());
    Font font;
    font.text = name;
    font.x = position.x();
    font.y = position.y();
    this->_fonts[id] = font;
}

bool QTVK2DViewImpl::exportGLTF(QString path)
{
    vtkNew<vtkGLTFExporter> writer;
    writer->SetFileName(path.toStdString().c_str());
    writer->InlineDataOn();
    writer->SetRenderWindow(this->_view->renderWindow());
    writer->Write();
    return true;
}

bool QTVK2DViewImpl::exportDXF(QString path)
{
    dx_iface* iface = new dx_iface();
    iface->cData = new dx_data();
    QList<std::shared_ptr<DRW_Line>> ls;
    if (_vtkActors.contains(ShapeType::Line)) {
        const QList<vtkProp3D*>& lines = _vtkActors[QTVK2DViewImpl::Line];
        for (vtkProp3D* prop : lines) {
            auto&& actor = static_cast<vtkActor*>(prop);
            vtkSmartPointer<vtkLineSource> lineSource = vtkLineSource::SafeDownCast(actor->GetMapper()->GetInputAlgorithm());
            if (lineSource) {
                double p1[3], p2[3], color[3];
                lineSource->GetPoint1(p1);
                lineSource->GetPoint2(p2);
                actor->GetProperty()->GetColor(color);

                std::shared_ptr<DRW_Line> line = std::make_shared<DRW_Line>();
                line->basePoint.x = p1[0];
                line->basePoint.y = p1[1];
                line->secPoint.x = p2[0];
                line->secPoint.y = p2[1];
                line->colorName = "red";
                line->lWeight = DRW_LW_Conv::lineWidth::width22;
                ls.push_back(line);
                iface->cData->mBlock->ent.push_back(line.get());
            }
        }
    }

    QList<std::shared_ptr<DRW_Point>> ps;
    for (const auto& point : *_cloud) {
        std::shared_ptr<DRW_Point> p = std::make_shared<DRW_Point>();
        p->basePoint.x = point.x;
        p->basePoint.y = point.y;
        // 红色
        p->color24 = 0xFF0000;
        p->colorName = "red";
        p->thickness = 10;
        p->lWeight = DRW_LW_Conv::lineWidth::width20;
        ps.push_back(p);
        iface->cData->mBlock->ent.push_back(p.get());
    }
    QList<std::shared_ptr<DRW_Text>> ts;
    for (auto&& font : _fonts) {
        std::shared_ptr<DRW_Text> t = std::make_shared<DRW_Text>();
        t->height = 2;
        t->alignH = DRW_Text::HCenter;
        t->alignV = DRW_Text::VMiddle;
        t->text = font.text.toLocal8Bit();
        t->basePoint.x = font.x;
        t->basePoint.y = font.y;

        t->secPoint.x = font.x;
        t->secPoint.y = font.y;
        t->color24 = 0xFF0000;
        ts.push_back(t);
        iface->cData->mBlock->ent.push_back(t.get());
    }

    return iface->fileExport(path.toStdString(), DRW::AC1009, false, iface->cData, false);
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

    this->_view->renderWindow()->GetRenderers()->GetFirstRenderer()->AddActor(actor);

    if (a == b) {
        this->_vtkActors[ShapeType::Circle].push_back(actor);
    } else {
        this->_vtkActors[ShapeType::Oval].push_back(actor);
    }
}

void QTVK2DViewImpl::addCircle(Eigen::Vector2d center, double radius, QColor color)
{
    this->addOval(center, radius, radius, 0, color);
}

void QTVK2DViewImpl::amplifyOval(int emplifySacle)
{
    for (auto&& actor : this->_vtkActors[ShapeType::Oval]) {
        double* scale = actor->GetScale();
        actor->SetScale(emplifySacle, emplifySacle, 1);
    }
    this->refresh();
}

void QTVK2DViewImpl::clear()
{
    _cloud->clear();
    for (auto&& type : this->_vtkActors) {
        for (auto&& actor : type) {
            this->_view->renderWindow()->GetRenderers()->GetFirstRenderer()->RemoveActor(actor);
        }
    }
    for (auto&& id : _fonts.keys()) {
        this->_viewer->removeShape(id.toStdString());
    }
    this->_fonts.clear();
    this->_vtkActors.clear();
    this->_fontSize = 50;
    this->refresh();
}

void QTVK2DViewImpl::setCameraBaseOnCloud()
{
    if (this->_cloud->size() <= 1) {
        return;
    }
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
    if (std::isnan(xMin) || std::isnan(xMax) || std::isnan(yMin) || std::isnan(yMax)) {
        return;
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
    this->_view->renderWindow()->GetRenderers()->GetFirstRenderer()->ResetCameraClippingRange();
}

void QTVK2DViewImpl::setCameraBaseOnVtkPoint()
{
    if (_vtkActors[Point].isEmpty()) {
        return; // 无 Actor 直接返回
    }

    // 初始化包围盒范围
    double xMin = VTK_DOUBLE_MAX, xMax = VTK_DOUBLE_MIN;
    double yMin = VTK_DOUBLE_MAX, yMax = VTK_DOUBLE_MIN;

    // 遍历所有 Actor 合并包围盒
    auto function = [](double& xMin, double& xMax, double& yMin, double& yMax, QList<vtkProp3D*> props) {
        for (auto&& prop : props) {
            if (!prop) {
                continue;
            }
            // 获取 Actor 的包围盒 [xmin, xmax, ymin, ymax, zmin, zmax]
            double bounds[6];
            prop->GetBounds(bounds);
            // 更新全局范围
            xMin = std::min(xMin, bounds[0]);
            xMax = std::max(xMax, bounds[1]);
            yMin = std::min(yMin, bounds[2]);
            yMax = std::max(yMax, bounds[3]);
        }
    };
    function(xMin, xMax, yMin, yMax, _vtkActors[Point]);
    function(xMin, xMax, yMin, yMax, _vtkActors[Triangle]);

    // 检查包围盒有效性
    if (xMin > xMax || yMin > yMax) {
        return; // 无效包围盒
    }

    // 计算场景中心点
    const double centerX = (xMin + xMax) / 2.0;
    const double centerY = (yMin + yMax) / 2.0;
    this->_center = Eigen::Vector2d(centerX, centerY);

    // 计算相机高度（基于最大跨度）
    const double xSpan = xMax - xMin;
    const double ySpan = yMax - yMin;
    const double maxSpan = std::max(xSpan, ySpan);

    // 设置相机参数
    vtkCamera* camera = this->_camera;
    camera->SetPosition(centerX, centerY, maxSpan); // 正上方视角
    camera->SetFocalPoint(centerX, centerY, 0); // 聚焦场景中心
    // 更新裁剪范围
    this->_view->renderWindow()->GetRenderers()->GetFirstRenderer()->ResetCameraClippingRange();
}

bool QTVK2DViewImpl::eventFilter(QObject* obj, QEvent* event)
{
    // 监听键盘按键C
    if (event->type() == QEvent::KeyPress) {
        if (auto&& keyEvent = dynamic_cast<QKeyEvent*>(event)) {
            // font down
            if (keyEvent->key() == Qt::Key_Down && keyEvent->modifiers() == Qt::ControlModifier) {
                qDebug() << "font down";
                if (_fontSize - FONT_SCALE_STEP <= 0) {
                    return false;
                }
                _fontSize -= FONT_SCALE_STEP;
                for (auto&& id : this->_fonts.keys()) {
                    auto&& x = _fonts[id].x;
                    auto&& y = _fonts[id].y;
                    this->addFont(id, Eigen::Vector2d(x, y), id);
                }
                this->refresh();
                return true;
            }
            // font up
            else if (keyEvent->key() == Qt::Key_Up && keyEvent->modifiers() == Qt::ControlModifier) {
                qDebug() << "font up";
                _fontSize += FONT_SCALE_STEP;
                for (auto&& id : this->_fonts.keys()) {
                    auto&& x = _fonts[id].x;
                    auto&& y = _fonts[id].y;
                    this->addFont(id, Eigen::Vector2d(x, y), id);
                }
                this->refresh();
                return true;
            }
        }
    }
    return false;
}

void QTVK2DViewImpl::updateTriangle()
{
    int* winSize = this->_view->renderWindow()->GetSize();
    if (winSize[0] <= 0 || winSize[1] <= 0)
        return;
    // 三角形占窗口高度的3% 转为像素高度
    double radius = this->windowHeight2Pixel(winSize[1] * 0.03);

    for (auto&& prop : this->_vtkActors[Triangle]) {
        auto&& actor = static_cast<vtkActor*>(prop);
        auto&& mapper = actor->GetMapper();
        vtkSmartPointer<vtkRegularPolygonSource> triangleSource = vtkRegularPolygonSource::SafeDownCast(mapper->GetInputConnection(0, 0)->GetProducer());
        triangleSource->SetRadius(radius);
        triangleSource->Update();
    }
    this->refresh();
}

double QTVK2DViewImpl::windowHeight2Pixel(double height)
{
    // 获取窗口尺寸
    int* winSize = this->_view->renderWindow()->GetSize();
    if (winSize[0] <= 0 || winSize[1] <= 0)
        return 0;
    int windowHeight = winSize[1];
    // 获取相机参数
    double viewAngle = _camera->GetViewAngle(); // 垂直视场角（度）
    double distance = _camera->GetDistance(); // 相机到焦点的距离

    // 计算 窗口高度/像素高度 的比例系数
    double pixelUnit;
    if (_camera->GetParallelProjection()) {
        // 正交投影
        double parallelScale = _camera->GetParallelScale();
        pixelUnit = windowHeight / (2.0 * parallelScale);
    } else {
        // 透视投影
        double tanHalfAngle = tan(vtkMath::RadiansFromDegrees(viewAngle / 2.0));
        pixelUnit = (windowHeight / 2.0) / (distance * tanHalfAngle);
    }
    return height / pixelUnit;
}
