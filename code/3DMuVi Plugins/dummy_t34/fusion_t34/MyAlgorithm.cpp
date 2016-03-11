#include "algorithm.h"
#include "workflow/workflow/datapackets/CDataFeature.h"
#include <QThread>

//----------------------------------------
// Adjust these functions to your needs
//----------------------------------------

void _CLASS_GEN(Algorithm)::OnInitialize(){
}

bool _CLASS_GEN(Algorithm)::ValidateParameters(const QJsonObject *params) const{
    Q_UNUSED(params);
    return true;
}

void _CLASS_GEN(Algorithm)::executeAlgorithm(CContextDataStore *store){
    Q_UNUSED(store);
    QThread::sleep(2);
}
