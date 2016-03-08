#include "algorithm.h"
#include "workflow/workflow/datapackets/CDataFeature.h"

//----------------------------------------
// Adjust these functions to your needs
//----------------------------------------

void CLASS_GEN(Algorithm)::OnInitialize()
{
  mOutputTypes.push_back(DT_FEATURE_MATCH);
  mInputTypes.push_back(DT_INPUTIMAGES);
}

bool CLASS_GEN(Algorithm)::ValidateParameters(const QJsonObject* params) const
{
  if(params && params->value("parameter").type() == QJsonValue::Double
     && params->value("parameter").toInt() < 50)
  {
    return true;
  }
  return false;
}

void CLASS_GEN(Algorithm)::executeAlgorithm(CContextDataStore* store)
{
  Q_UNUSED(store);
}
