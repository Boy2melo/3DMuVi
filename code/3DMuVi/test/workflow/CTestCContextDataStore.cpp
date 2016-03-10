#include "CTestCContextDataStore.h"
#include "workflow/workflow/ccontextdatastore.h"
#include "io/CInputDataSet.h"
#include "workflow/workflow/datapackets/CDataFeature.h"
#include "CTestCFeatureView.h"

void CTestCContextDataStore::testInitializeFromStorage() {
    CContextDataStore store;
    auto imagepacket = new CInputDataSet(); //Will be cleaned by store destructor

    store.InitializeFromStorage(imagepacket);

    QCOMPARE(store.getData<CInputDataSet>().get(), imagepacket);
}

void CTestCContextDataStore::testDataAccess() {
    CContextDataStore store;
    auto packet1 = std::make_shared<CDataFeature>();
    std::shared_ptr<CInputDataSet> packet2 = store.createData<CInputDataSet>();
    auto packet3 = std::make_shared<CDataFeature>();

    store.appendData(std::shared_ptr<CDataFeature>(packet1));

    QCOMPARE(store.getData<CDataFeature>(), packet1);
    QCOMPARE(store.getData<CInputDataSet>(), packet2);

    store.appendData(packet3, false);

    QCOMPARE(store.getData<CDataFeature>(), packet1);

    store.appendData(packet3, true);

    QCOMPARE(store.getData<CDataFeature>(), packet3);
}

void CTestCContextDataStore::testAbortFlag() {
    CContextDataStore store;

    QCOMPARE(store.IsAborted(), false);

    store.SetIsAborted(true);

    QCOMPARE(store.IsAborted(), true);

    store.SetIsAborted(false);

    QCOMPARE(store.IsAborted(), false);
}

void CTestCContextDataStore::testCalculateStep() {
    CContextDataStore store;

    QCOMPARE(store.getCurrentCalculationStep(), -1);

    store.incCalculationStep();

    QCOMPARE(store.getCurrentCalculationStep(), 0);

    store.resetCalculationStep();

    QCOMPARE(store.getCurrentCalculationStep(), -1);
}

void CTestCContextDataStore::testApplyToDataView() {
    CContextDataStore store;
    std::shared_ptr<CDataFeature> packet = store.createData<CDataFeature>();
    bool raised;
    CTestCFeatureView view(packet.get(), &raised);

    store.ApplyToDataView(&view);

    if(!raised) {
        QFAIL("The view was not triggered");
    }
}
