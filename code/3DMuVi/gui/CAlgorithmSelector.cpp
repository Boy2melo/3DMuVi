#include <QVBoxLayout>
#include <QGroupBox>
#include <QMessageBox>

#include <workflow/plugin/cpluginmanager.h>

#include "CStepComboBox.h"

#include "CAlgorithmSelector.h"

//============================================================
/*!
@param workflow
 */
 //============================================================
CAlgorithmSelector::CAlgorithmSelector(QWidget *parent) : QWidget(parent) {
    setLayout(new QVBoxLayout);
}

//============================================================
/*!
 */
 //============================================================
CAlgorithmSelector::~CAlgorithmSelector() {

}

//============================================================
/*!
@param workflow
 */
 //============================================================
void CAlgorithmSelector::setWorkflow(AWorkflow& workflow) {
    uint32_t steps = workflow.getStepCount();
    QPushButton* start = new QPushButton("Start", this);
    mStatus = new QStatusBar(start);
    mStatus->showMessage("Choose Images");
    mStartButton = start;
    start->setDisabled(true);
    connect(start, &QPushButton::clicked, this, &CAlgorithmSelector::startButtonPushed);

    if(mpWorkflow != nullptr)
    {
      disconnect(mpWorkflow, &AWorkflow::sigDataStoreFinished, this, &CAlgorithmSelector::onDataStoreFinished);
    }

    mpWorkflow = &workflow;

    connect(mpWorkflow, &AWorkflow::sigDataStoreFinished, this, &CAlgorithmSelector::onDataStoreFinished);

    QLayoutItem *child;
    while ((child = layout()->takeAt(0)) != 0) {
      if(child->widget())
      {
        delete child->widget();
      }
        delete child;
    }

    for (uint i = 0; i < steps; i++) {
        QStringList plugins = workflow.getAvailablePlugins(i);

        QGroupBox* groupBox = new QGroupBox(workflow.getAlgorithmType(i), this);
        CStepComboBox* comboBox = new CStepComboBox(i, groupBox);
        for (QString p : plugins) {
            comboBox->addItem(p);
        }
        groupBox->setLayout(new QVBoxLayout);
        groupBox->layout()->addWidget(comboBox);
        layout()->addWidget(groupBox);
        connect(comboBox,
                static_cast<void(CStepComboBox::*)(int)>(&CStepComboBox::currentIndexChanged), this,
                &CAlgorithmSelector::onCurrentIndexChanged);
        if (plugins.size() > 0) {
            workflow.trySetStep(i, comboBox->currentText());
        }
    }

    layout()->addWidget(start);
    layout()->addWidget(mStatus);
}

void CAlgorithmSelector::setDataStore(const QString& storeId) {
    mDataStoreId = storeId;
    mStartButton->setDisabled(false);
    mStatus->showMessage("Ready to Start");
}

//============================================================
/*!
@param index
 */
 //============================================================
void CAlgorithmSelector::onCurrentIndexChanged(int index) {
    CStepComboBox* sendingComboBox = qobject_cast<CStepComboBox*>(sender());
    int step = sendingComboBox->getStep();

    bool success = mpWorkflow->trySetStep(step, sendingComboBox->itemText(index));

    if (success) {
      emit algorithmChanged(step);
    }
}

void CAlgorithmSelector::startButtonPushed(bool isPushed) {
    Q_UNUSED(isPushed);
    emit start();
    QPushButton* startStopButton = nullptr;

    int steps = mpWorkflow->getStepCount();
    for (int i = 0; i < steps; i++) {
        if (mpWorkflow->getStep(i) == nullptr) {
            QMessageBox::warning(this, "Warning", "Please choose algorithms for the steps in the workflow.");

            return;
        }
    }

    mStatus->showMessage("Running...");

    startStopButton = qobject_cast<QPushButton*>(sender());
    if (startStopButton) {
        if (startStopButton->text() == "Stop") {
            startStopButton->setText("Start");
            emit workflowRunning(false);
        } else if (startStopButton->text() == "Start") {
            startStopButton->setText("Stop");
            emit workflowRunning(true);
            if (!mpWorkflow->run(mDataStoreId)) {
                startStopButton->setText("Start");
                mStatus->showMessage("Error occurred");
                emit workflowRunning(false);
            }
        }
    }

}

void CAlgorithmSelector::onDataStoreFinished(CContextDataStore* dataStore) {
    QPushButton* startStopButton = findChild<QPushButton*>();

    if (dataStore->getId() == mDataStoreId) {
        startStopButton->setText("Start");
        emit workflowRunning(false);
    }

    if(dataStore->isAborted())
    {
      mStatus->showMessage("Error: Failed to compute the workflow");
    }
    else{
      mStatus->showMessage("Workflow finished.");
    }
}
