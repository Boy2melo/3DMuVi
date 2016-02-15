//--- main.cpp - start ---
#include "gui/CMainWindow.h"
#include <QApplication>
#include "workflow/workflow/cworkflowmanager.h"
#include "workflow/plugin/cpluginmanager.h"

int main( int argc, char* argv[])
{
    QApplication a(argc, argv);

    CPluginManager::Instance();
    CWorkflowManager::Instance();

    CMainWindow w;
    w.show();
    return a.exec();
}

//--- main.cpp - end ---
