#include <QTest>
#include "CTestCFeatureView.h"

CTestCFeatureView::~CTestCFeatureView() {}

CTestCFeatureView::CTestCFeatureView(CDataFeature* feature, bool* raised) {
    mFeature = feature;
    mRaised = raised;
}

void CTestCFeatureView::applyData(CDataFeature const* feature) {
    QCOMPARE(feature, mFeature);
    *mRaised = true;
}
