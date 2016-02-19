#ifndef CSTEPCOMBOBOX_H
#define CSTEPCOMBOBOX_H

#include <QComboBox>

/*!
 * \brief Derived class from QComboBox with added member variable mStep.
 */
class CStepComboBox : public QComboBox
{
  Q_OBJECT

public:
  explicit CStepComboBox(int step, QWidget* parent = nullptr);

  int getStep();

private:
  int mStep;
};

#endif // CSTEPCOMBOBOX_H
