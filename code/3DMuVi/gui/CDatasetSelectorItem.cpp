#include "CDatasetSelectorItem.h"

CDatasetSelectorItem::CDatasetSelectorItem(CContextDataStore* dataStore, QString text)
  : QListWidgetItem(text), mpDataStore(dataStore)
{
}

CContextDataStore* CDatasetSelectorItem::getDataStore()
{
  return mpDataStore;
}
