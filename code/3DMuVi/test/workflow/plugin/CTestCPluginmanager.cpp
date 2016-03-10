#include "CTestCPluginmanager.h"
#include "workflow/plugin/cpluginmanager.h"

void CTestCPluginmanager::testInstance() {
    auto manager1 = CPluginManager::Instance();
    auto manager2 = CPluginManager::Instance();

    QCOMPARE(manager1, manager2);
}


void CTestCPluginmanager::testInitialize() {
    auto manager = CPluginManager::Instance();
    auto result = manager->Initialize();

    if(result < 0) {
        QFAIL("Initialisierung des Pluginmanagers schlug fehl");
    }
}


void CTestCPluginmanager::testInstanciate() {
    auto manager = CPluginManager::Instance();

    auto plugins = manager->getPlugins();
    // Nothing to check, just increase coverage and catch exceptions
}
