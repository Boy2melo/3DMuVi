#include "algorithm.h"

//----------------------------------------
// Adjust these functions to your needs
//----------------------------------------

void CLASS_GEN(Algorithm)::OnInitialize(){
    mInputTypes.push_back(DT_FEATURE_MATCH);
    mInputTypes.push_back(DT_INPUTIMAGES);
}

bool CLASS_GEN(Algorithm)::ValidateParameters(const QJsonObject *params) const{
    // First level typechecks are already done, see plugin.cpp

}

void CLASS_GEN(Algorithm)::executeAlgorithm(CContextDataStore *store){
    //do stuff
}
