#include "CTestCDatasetSelector.h"

#include <QTest>
#include <QPushButton>

#include <gui/CDatasetSelector.h>

const unsigned int CTestCDatasetSelector::N_TEST_DATA_STORES = 5;

//needs to be smaller than N_TEST_DATA_STORES
const unsigned int CTestCDatasetSelector::TEST_DATA_STORE_INDEX = 2;

void CTestCDatasetSelector::initTestCase()
{
  mpDataStores = new CContextDataStore*[N_TEST_DATA_STORES];
  for(unsigned int i = 0; i < N_TEST_DATA_STORES; i++)
  {
    mpDataStores[i] = new CContextDataStore;
  }
}

void CTestCDatasetSelector::newDataStore()
{
  CDatasetSelector selector;
  auto dataStore = new CContextDataStore;

  QListWidget* listWidget = selector.findChild<QListWidget*>();
  QVERIFY2(listWidget != nullptr, "List widget not found.");

  selector.newDataStore(dataStore);
  auto item = listWidget->item(0);
  QVERIFY(item != nullptr);
  QCOMPARE(item->text(), dataStore->getId());

  delete dataStore;
}

void CTestCDatasetSelector::clear()
{
  CDatasetSelector selector;

  QListWidget* listWidget = selector.findChild<QListWidget*>();
  QVERIFY2(listWidget != nullptr, "List widget not found.");

  for(unsigned int i = 0; i < N_TEST_DATA_STORES; i++)
  {
    selector.newDataStore(mpDataStores[i]);
    QCOMPARE(listWidget->count(), static_cast<int>(i + 1));
  }

  selector.clear();
  QCOMPARE(listWidget->count(), 0);

  //delete dataStore;
}

void CTestCDatasetSelector::currentDataStoreChanged()
{
  CDatasetSelector selector;

  QListWidget* listWidget = selector.findChild<QListWidget*>();
  QVERIFY2(listWidget != nullptr, "List widget not found.");

  for(unsigned int i = 0; i < N_TEST_DATA_STORES; i++)
  {
    selector.newDataStore(mpDataStores[i]);
    QCOMPARE(listWidget->count(), static_cast<int>(i + 1));
  }

  mCurrentDataStoreChangedEmitted = false;

  connect(&selector, &CDatasetSelector::currentDataStoreChanged,
          this, &CTestCDatasetSelector::currentDataStoreChangedVerifier);
  listWidget->setCurrentRow(TEST_DATA_STORE_INDEX);
  QVERIFY(mCurrentDataStoreChangedEmitted);
}

void CTestCDatasetSelector::dataStoreDeleted()
{
  CDatasetSelector selector;

  QListWidget* listWidget = selector.findChild<QListWidget*>();
  QVERIFY2(listWidget != nullptr, "List widget not found.");

  for(unsigned int i = 0; i < N_TEST_DATA_STORES; i++)
  {
    selector.newDataStore(mpDataStores[i]);
    QCOMPARE(listWidget->count(), static_cast<int>(i + 1));
  }

  listWidget->setCurrentRow(TEST_DATA_STORE_INDEX);

  mDataStoreDeletedEmitted = false;

  connect(&selector, &CDatasetSelector::dataStoreDeleted,
          this, &CTestCDatasetSelector::dataStoreDeletedVerifier);

  QPushButton* removeButton = nullptr;
  for(auto pb : selector.findChildren<QPushButton*>())
  {
    if(pb->text() == "Remove")
    {
      removeButton = pb;
      break;
    }
  }
  QVERIFY2(removeButton != nullptr, "Remove button not found.");

  removeButton->clicked();

  QVERIFY(mDataStoreDeletedEmitted);
}

void CTestCDatasetSelector::newDataStoreRequested()
{
  CDatasetSelector selector;

  mNewDataStoreRequestedEmitted = false;

  connect(&selector, &CDatasetSelector::newDataStoreRequested,
          this, &CTestCDatasetSelector::newDataStoreRequestedVerifier);

  QPushButton* addButton = nullptr;
  for(auto pb : selector.findChildren<QPushButton*>())
  {
    if(pb->text() == "Add")
    {
      addButton = pb;
      break;
    }
  }
  QVERIFY2(addButton != nullptr, "Remove button not found.");

  addButton->clicked();

  QVERIFY(mNewDataStoreRequestedEmitted);
}

void CTestCDatasetSelector::cleanupTestCase()
{
  for(unsigned int i = 0; i < N_TEST_DATA_STORES; i++)
  {
    delete mpDataStores[i];
  }
  delete [] mpDataStores;
}

void CTestCDatasetSelector::currentDataStoreChangedVerifier(CContextDataStore* dataStore)
{
  mCurrentDataStoreChangedEmitted = true;
  QCOMPARE(dataStore, mpDataStores[TEST_DATA_STORE_INDEX]);
}

void CTestCDatasetSelector::dataStoreDeletedVerifier(CContextDataStore* dataStore)
{
  mDataStoreDeletedEmitted = true;
  QCOMPARE(dataStore, mpDataStores[TEST_DATA_STORE_INDEX]);
}

void CTestCDatasetSelector::newDataStoreRequestedVerifier()
{
  mNewDataStoreRequestedEmitted = true;
}
