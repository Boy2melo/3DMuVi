#ifndef CPLUGINMANAGER_H
#define CPLUGINMANAGER_H

#include <QVector>
#include "workflow/plugin/aplugin.h"

class CPluginManager
{
private:
    CPluginManager();

public:
    static CPluginManager* Instance();
    qint32 Initialize();
    QVector<APlugin*> getPlugins();
    QVector<APlugin*> getPlugins(QString type);
};

#endif // CPLUGINMANAGER_H
