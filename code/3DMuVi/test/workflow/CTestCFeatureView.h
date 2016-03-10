#pragma once
#include "workflow/workflow/datapackets/CDataFeature.h"
#include "workflow/workflow/idataview.h"

class CTestCFeatureView : public IDataView{
    CDataFeature *mFeature;
    bool *mRaised;
public:
    CTestCFeatureView(CDataFeature *feature, bool *raised);
    ~CTestCFeatureView();


    void applyData(CDataFeature const*) override;
};

