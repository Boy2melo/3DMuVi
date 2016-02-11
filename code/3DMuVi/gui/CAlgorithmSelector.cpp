#include <QVBoxLayout>
#include <QGroupBox>
#include <QPushButton>
#include <QMessageBox>

#include <workflow/plugin/cpluginmanager.h>

#include "CStepComboBox.h"

#include "CAlgorithmSelector.h"

//============================================================
/*!
@param workflow
 */
//============================================================
CAlgorithmSelector::CAlgorithmSelector(QWidget *parent) : QWidget(parent)
{
  setLayout(new QVBoxLayout);
}

//============================================================
/*!
 */
//============================================================
CAlgorithmSelector::~CAlgorithmSelector()
{

}

//============================================================
/*!
@param workflow
 */
//============================================================
void CAlgorithmSelector::setWorkflow(AWorkflow& workflow)
{
  uint32_t steps = workflow.getStepCount();
  QPushButton* start = new QPushButton("Start", this);
  connect(start, &QPushButton::clicked, this, &CAlgorithmSelector::startButtonPushed);


  mpWorkflow = &workflow;

  QLayoutItem *child;
  while ((child = layout()->takeAt(0)) != 0) {

      delete child;
  }

  for(int i=0; i < steps; i++)
  {
    const QVector<IPlugin*> plugins = CPluginManager::Instance()->getPlugins(workflow.getAlgorithmType(i));

    QGroupBox* groupBox = new QGroupBox(workflow.getAlgorithmType(i), this);
    CStepComboBox* comboBox = new CStepComboBox(i, groupBox);
      for(IPlugin* p : plugins)
      {
        comboBox->addItem(p->Autor(), QVariant::fromValue((void*) p));
      }
    groupBox->setLayout(new QVBoxLayout);
    groupBox->layout()->addWidget(comboBox);
    layout()->addWidget(groupBox);
    layout()->addWidget(start);
  }
}

void CAlgorithmSelector::setDataStore(const QString& storeId)
{

}

//============================================================
/*!
@param index
 */
//============================================================
void CAlgorithmSelector::onCurrentIndexChanged(int index)
{
  CStepComboBox* sendingComboBox = qobject_cast<CStepComboBox*>(sender());
  int step = sendingComboBox->getStep();
  IPlugin* plugin = static_cast<IPlugin*>(sendingComboBox->itemData(index).value<void*>());

  if(plugin)
  {
    bool success = mpWorkflow->trySetStep(step, plugin);

    if(success)
    {
      emit algorithmChanged(step);
    }
  }
}

void CAlgorithmSelector::startButtonPushed(bool isPushed)
{
  QPushButton* startStopButton = nullptr;

  int steps = mpWorkflow->getStepCount();
  for(int i=0; i < steps; i++)
  {
    if(mpWorkflow->getStep(i) == nullptr)
    {
      QMessageBox::warning(this, "Warning","Please choose algorithms for the steps in the workflow.");

      return;
    }
  }

  emit workflowRunning(true);

  mpWorkflow->run(mDataStoreId);

  startStopButton = qobject_cast<QPushButton*>(sender());
  if(startStopButton)
  {
    startStopButton->setText("Stop");
  }
}
