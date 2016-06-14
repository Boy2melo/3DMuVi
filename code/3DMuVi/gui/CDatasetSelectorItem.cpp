#include "CDatasetSelectorItem.h"

CDatasetSelectorItem::CDatasetSelectorItem(CContextDataStore* dataStore, QString text)
  : QListWidgetItem(text), mpDataStore(dataStore)
{
  setFlags(flags() | Qt::ItemIsEditable);
}

CContextDataStore* CDatasetSelectorItem::getDataStore()
{
  return mpDataStore;
}
