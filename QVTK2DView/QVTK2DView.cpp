#include "QVTK2DView.h"

#include "QTVK2DViewImpl.h"

QVTK2DView* QVTK2DView::New(QWidget* parent)
{
    return new QTVK2DViewImpl(parent);
}

QVTK2DView::~QVTK2DView()
{
}

QVTK2DView::QVTK2DView(QWidget* parent)
{
}