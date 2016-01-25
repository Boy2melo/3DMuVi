#ifndef CALGORITHMSELECTOR_H
#define CALGORITHMSELECTOR_H

#include <QWidget>

#include <workflow/workflow/aworkflow.h>

class CAlgorithmSelector : public QWidget
{
public:
  CAlgorithmSelector(AWorkflow& workflow);
  void setWorkflow(AWorkflow& workflow);

public slots:
  void algorithmChanged(int step);
};

#endif // CALGORITHMSELECTOR_H
