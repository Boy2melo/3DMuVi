#ifndef CALGORITHMSELECTOR_H
#define CALGORITHMSELECTOR_H

#include <QWidget>

#include <workflow/workflow/aworkflow.h>

class CAlgorithmSelector : public QWidget
{
  Q_OBJECT

public:
  explicit CAlgorithmSelector(QWidget *parent);
  ~CAlgorithmSelector();
  void setWorkflow(AWorkflow& workflow);
  void setDataStore(const QString& storeId);

signals:
  void algorithmChanged(int step);
  void workflowRunning(bool isRunning);

private:
  AWorkflow* mpWorkflow = nullptr;
  QString mDataStoreId;
private slots:
  void onCurrentIndexChanged(int index);
  void startButtonPushed(bool isPushed);
};

#endif // CALGORITHMSELECTOR_H
