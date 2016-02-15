#include "exampledata.h"
#include "workflow/workflow/ccontextdatastore.h"


ExampleData::ExampleData() {}


ExampleData::~ExampleData() {}


QString ExampleData::getDataType() const {
    return CContextDataStore::DT_POSE;
}

void ExampleData::getMyData() {
}

void ExampleData::setMyData() {
}
