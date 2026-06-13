// CommonUtils module global definitions and exports
#ifndef COMMONUTILS_GLOBAL_H
#define COMMONUTILS_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(COMMONUTILS_LIBRARY)
#  define COMMONUTILSSHARED_EXPORT Q_DECL_EXPORT
#else
#  define COMMONUTILSSHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // COMMONUTILS_GLOBAL_H
