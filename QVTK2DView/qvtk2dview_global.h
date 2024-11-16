#pragma once

#include <QtCore/qglobal.h>

#ifndef BUILD_STATIC
# if defined(QVTK2DVIEW_LIB)
#  define QVTK2DVIEW_EXPORT Q_DECL_EXPORT
# else
#  define QVTK2DVIEW_EXPORT Q_DECL_IMPORT
# endif
#else
# define QVTK2DVIEW_EXPORT
#endif
