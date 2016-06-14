#ifndef CTESTALGORITHMSELECTION_H
#define CTESTALGORITHMSELECTION_H

#include <gui/CAlgorithmSelector.h>
#include <workflow/workflow/fourphase/cfourphaseworkflow.h>

#include "gui/CStepComboBox.h"

class CTestAlgorithmSelection : public QObject
{
    Q_OBJECT

private slots:
    void test();
};


#endif // CTESTALGORITHMSELECTION_H
