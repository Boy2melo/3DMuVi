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

signals:
  void algorithmChanged(int step);

private:
  AWorkflow* mpWorkflow = nullptr;
private slots:
  void onCurrentIndexChanged(int index);
};

#endif // CALGORITHMSELECTOR_H
