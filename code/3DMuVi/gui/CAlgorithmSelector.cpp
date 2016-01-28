#include <QVBoxLayout>
#include <QGroupBox>

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

  mpWorkflow = &workflow;

  for(int i=0; i< steps; i++)
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
  }
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
