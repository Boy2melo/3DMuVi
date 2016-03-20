#ifndef CTESTCDATASETSELECTOR_H
#define CTESTCDATASETSELECTOR_H

#include <QObject>

#include <workflow/workflow/ccontextdatastore.h>

/*!
\brief Test for CDatasetSelector.
\author Stefan Wolf

This class is a unit test for CDatasetSelector.
*/
class CTestCDatasetSelector : public QObject
{
  Q_OBJECT

private:
  const static unsigned int N_TEST_DATA_STORES;
  const static unsigned int TEST_DATA_STORE_INDEX;

  bool mCurrentDataStoreChangedEmitted = false;
  bool mDataStoreDeletedEmitted = false;
  bool mNewDataStoreRequestedEmitted = false;
  CContextDataStore** mpDataStores = nullptr;

private slots:
  void initTestCase();

  /*!
  \brief Tests newDataStore.

  Adds a new CContextDataStore to a CDatasetSelector and checks if it has been added properly.
  */
  void newDataStore();

  /*!
  \brief Tests clear.

  Adds some CContextDataStores to a CDatasetSelector, calls clear() and checks if any items are
  left in the list.
  */
  void clear();

  /*!
  \brief Tests currentDataStoreChanged.

  Adds some CContextDataStores to a CDatasetSelector, selects one of them and checks if the signal
  has been emitted properly.
  */
  void currentDataStoreChanged();

  /*!
  \brief Tests dataStoreDeleted.

  Adds some CContextDataStores to a CDatasetSelector, selects one of them, clicks remove and
  checks if the signal has been emitted properly.
  */
  void dataStoreDeleted();

  /*!
  \brief Tests newDataStoreRequested.

  Clicks add and checks it the signal has been emitted properly.
  */
  void newDataStoreRequested();

  void cleanupTestCase();

protected slots:
  /*!
  \brief This slots is intended for internal use and shouldn't be used.

  It can't be private since all private slots are executed as tests.
  */
  void currentDataStoreChangedVerifier(CContextDataStore* dataStore);
  /*!
  \brief This slots is intended for internal use and shouldn't be used.

  It can't be private since all private slots are executed as tests.
  */
  void dataStoreDeletedVerifier(CContextDataStore* dataStore);
  /*!
  \brief This slots is intended for internal use and shouldn't be used.

  It can't be private since all private slots are executed as tests.
  */
  void newDataStoreRequestedVerifier();
};

#endif // CTESTCDATASETSELECTOR_H
