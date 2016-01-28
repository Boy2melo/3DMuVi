#include "CStepComboBox.h"


CStepComboBox::CStepComboBox(int step, QWidget* parent) : QComboBox(parent), mStep(step)
{
}

int CStepComboBox::getStep()
{
  return mStep;
}
