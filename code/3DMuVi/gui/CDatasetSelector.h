#ifndef CDATASETSELECTOR_H
#define CDATASETSELECTOR_H

#include <QListWidget>

#include <io/CInputDataSet.h>
#include <workflow/workflow/ccontextdatastore.h>

namespace Ui {
  class CDatasetSelector;
}

/*!
\brief A widget for seleting, creating and removing data sets.
\author Stefan Wolf
*/
class CDatasetSelector : public QWidget
{
  Q_OBJECT

public:
  /*!
  \brief Initializes the user interface objects and connects necessary signals and slots.
  \param parent The parent widget of this one.
  */
  explicit CDatasetSelector(QWidget* parent = nullptr);
  /*!
  \brief Cleans up the user interface objects.
  */
  ~CDatasetSelector();

  /*!
  \brief Adds a context store to the list with the stores' id as text.
  \param dataStore The data store to add.
  */
  void newDataStore(CContextDataStore* dataStore);
  /*!
  \brief Clears the list of data stores.
  */
  void clear();

signals:
  /*!
  \brief Emitted when the user selects a data store.
  \param dataStore The selected data store.
  */
  void currentDataStoreChanged(CContextDataStore* dataStore);
  /*!
  \brief Emitted when the user removes a data store.
  \param dataStore The data store to be removed.
  */
  void dataStoreDeleted(CContextDataStore* dataStore);
  /*!
  \brief Emitted when the user wants to create a new data store.
  */
  void newDataStoreRequested();

private:
  Ui::CDatasetSelector* ui;

private slots:
  void onCurrentItemChanged(QListWidgetItem *current, QListWidgetItem *previous);
  void addDataset();
  void removeDataset();
};

#endif // CDATASETSELECTOR_H
