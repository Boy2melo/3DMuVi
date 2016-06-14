#include "CDatasetSelector.h"

#include "CDatasetSelectorItem.h"

#include "ui_CDatasetSelector.h"

CDatasetSelector::CDatasetSelector(QWidget* parent) : QWidget(parent), ui(new Ui::CDatasetSelector)
{
  ui->setupUi(this);

  connect(ui->datasetListWidget, &QListWidget::currentItemChanged,
          this, &CDatasetSelector::onCurrentItemChanged);
  connect(ui->addButton, &QPushButton::clicked, this, &CDatasetSelector::addDataset);
  connect(ui->removeButton, &QPushButton::clicked, this, &CDatasetSelector::removeDataset);
}

CDatasetSelector::~CDatasetSelector()
{
  delete ui;
}

void CDatasetSelector::newDataStore(CContextDataStore* dataStore)
{
  auto item = new CDatasetSelectorItem(dataStore, dataStore->getId());
  ui->datasetListWidget->addItem(item);
  ui->datasetListWidget->setCurrentItem(item);
  ui->removeButton->setEnabled(true);
}

void CDatasetSelector::clear()
{
  ui->datasetListWidget->clear();
}

void CDatasetSelector::onCurrentItemChanged(QListWidgetItem* current, QListWidgetItem* previous)
{
  Q_UNUSED(previous);
  auto dataStore = current ? static_cast<CDatasetSelectorItem*>(current)->getDataStore() : nullptr;
  emit currentDataStoreChanged(dataStore);
}

void CDatasetSelector::addDataset()
{
  emit newDataStoreRequested();
}

void CDatasetSelector::removeDataset()
{
  auto currentItem = static_cast<CDatasetSelectorItem*>(ui->datasetListWidget->currentItem());

  if(currentItem)
  {
    emit dataStoreDeleted(currentItem->getDataStore());
    delete currentItem;
  }

  if(ui->datasetListWidget->count() == 0)
  {
    ui->removeButton->setEnabled(false);
  }
}
