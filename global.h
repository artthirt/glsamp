#ifndef GLOBAL_H
#define GLOBAL_H

#include <QString>

const QString config_dir("xml/");
const QString log_dir("logs_glsamp/");

#if (QT_VERSION <= QT_VERSION_CHECK(5, 0, 0))
#	define QT4
typedef qreal mat_type;
#else
#	define QT5
typedef float mat_type;
#endif

const int hi_version			= 0;
const int lo_version			= 1;

const QString version_string = QString("%1.%2").arg(hi_version).arg(lo_version);

#endif // GLOBAL_H
