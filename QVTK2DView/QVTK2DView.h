#pragma once

#include "qvtk2dview_global.h"

#include <QWidget>

#include <Eigen/Core>

#ifndef VTK_VIEWER
#define VTK_VIEWER []() { return GetAppPropertyPtr(GetQClassName(QVTK2DView), QVTK2DView); }()
#endif

class QVTK2DVIEW_EXPORT QVTK2DView : public QWidget {
    Q_OBJECT

public:
    QVTK2DView(QWidget* parent = nullptr);
    static QVTK2DView* New(QWidget* parent = nullptr);

    virtual ~QVTK2DView() = 0;

    virtual void addPoint(QString name, Eigen::Vector2d point, QColor color = { 0, 0, 0 }) = 0;

    virtual void addVtkPoint(QString name, Eigen::Vector2d point, QColor color = { 0, 0, 0 }) = 0;

    virtual void addLine(Eigen::Vector2d p1, Eigen::Vector2d p2, QColor color = { 0, 0, 0 }) = 0;

    virtual void addTriangle(QString name, Eigen::Vector2d center, QColor color = { 0, 0, 0 }) = 0;

    virtual void addFont(QString name, Eigen::Vector2d position, const QString& id, QColor color = { 255, 0, 0 }) = 0;

    virtual void addOval(Eigen::Vector2d center, double a, double b, double a_angle, QColor color = { 0, 0, 0 }) = 0;

    virtual void addCircle(Eigen::Vector2d center, double radius, QColor color = { 0, 0, 0 }) = 0;

    virtual void addDoubleLine(Eigen::Vector2d p1, Eigen::Vector2d p2, QColor color = { 0, 0, 0 }) = 0;

    virtual void amplifyOval(int sacle) = 0;

    virtual void clear() = 0;

    virtual void refresh() = 0;

    virtual bool exportGLTF(QString path) = 0;

    virtual bool exportDXF(QString path) = 0;

    virtual void setCameraBaseOnCloud() = 0;

    virtual void setCameraBaseOnVtkPoint() = 0;

    virtual void updateTriangle() = 0;
};
