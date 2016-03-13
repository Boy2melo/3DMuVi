#include "CTestTabLog.h"

#include <QTest>
#include <QCheckBox>

#include "logger/controll/CLogController.h"
#include "gui/CLogWidget.h"
#include "gui/CMainWindow.h"

CTestTabLog::CTestTabLog(QObject *parent) : QObject(parent)
{
}

void CTestTabLog::test()
{
    CMainWindow mw;
    CLogWidget* cLogWidget = nullptr;
    QCheckBox* qCheckBoxDebug = nullptr;
    QCheckBox* qCheckBoxInfo = nullptr;
    QCheckBox* qCheckBoxWarning = nullptr;
    QCheckBox* qCheckBoxError = nullptr;

    CLogController::instance().getHistory().clearHistory();
    CLogController::instance().setMinLoglevel(LOG_DEBUG);

    __LOG(LOG_DEBUG) QString("log debug test");
    __LOG(LOG_INFO) QString("log info test");
    __LOG(LOG_WARN) QString("log warning test");
    __LOG(LOG_ERROR) QString("log error test");

    for(QWidget* w : QApplication::allWidgets())
    {
        CLogWidget* tryCLogWidget = qobject_cast<CLogWidget*>(w);
        QCheckBox* tryQCheckBoxDebug = qobject_cast<QCheckBox*>(w);
        QCheckBox* tryQCheckBoxInfo = qobject_cast<QCheckBox*>(w);
        QCheckBox* tryQCheckBoxWarning = qobject_cast<QCheckBox*>(w);
        QCheckBox* tryQCheckBoxError = qobject_cast<QCheckBox*>(w);

        if(tryCLogWidget)
        {
            cLogWidget = tryCLogWidget;
        }

        if(tryQCheckBoxDebug && tryQCheckBoxDebug->text() == "Debug")
        {
            qCheckBoxDebug = tryQCheckBoxDebug;
        }

        if(tryQCheckBoxWarning && tryQCheckBoxWarning->text() == "Warning")
        {
            qCheckBoxWarning = tryQCheckBoxWarning;
        }

        if(tryQCheckBoxError && tryQCheckBoxError->text() == "Error")
        {
            qCheckBoxError = tryQCheckBoxError;
        }

        if(tryQCheckBoxInfo && tryQCheckBoxInfo->text() == "Info")
        {
            qCheckBoxInfo = tryQCheckBoxInfo;
        }
    }

    QVERIFY2(cLogWidget, "CLogWidget not found.");
    QVERIFY2(qCheckBoxDebug, "QCheckBoxDebug not found.");
    QVERIFY2(qCheckBoxInfo, "QCheckBoxInfo not found.");
    QVERIFY2(qCheckBoxError, "QCheckBoxError not found.");
    QVERIFY2(qCheckBoxWarning, "QCheckboxWarning not found.");

    qCheckBoxDebug->setCheckState(Qt::Checked);
    qCheckBoxError->setCheckState(Qt::Unchecked);
    qCheckBoxInfo->setCheckState(Qt::Unchecked);
    qCheckBoxWarning->setCheckState(Qt::Unchecked);
    QVERIFY(cLogWidget->toPlainText().contains("log debug test"));
    QVERIFY(cLogWidget->toPlainText().contains("DEBUG", Qt::CaseInsensitive));
    QVERIFY(!cLogWidget->toPlainText().contains("ERROR", Qt::CaseInsensitive));
    QVERIFY(!cLogWidget->toPlainText().contains("INFO", Qt::CaseInsensitive));
    QVERIFY(!cLogWidget->toPlainText().contains("WARNING", Qt::CaseInsensitive));

    qCheckBoxDebug->setCheckState(Qt::Unchecked);
    qCheckBoxError->setCheckState(Qt::Checked);
    qCheckBoxInfo->setCheckState(Qt::Unchecked);
    qCheckBoxWarning->setCheckState(Qt::Unchecked);
    QVERIFY(cLogWidget->toPlainText().contains("log error test"));
    QVERIFY(!cLogWidget->toPlainText().contains("DEBUG", Qt::CaseInsensitive));
    QVERIFY(cLogWidget->toPlainText().contains("ERROR", Qt::CaseInsensitive));
    QVERIFY(!cLogWidget->toPlainText().contains("INFO", Qt::CaseInsensitive));
    QVERIFY(!cLogWidget->toPlainText().contains("WARNING", Qt::CaseInsensitive));

    qCheckBoxDebug->setCheckState(Qt::Unchecked);
    qCheckBoxError->setCheckState(Qt::Unchecked);
    qCheckBoxInfo->setCheckState(Qt::Checked);
    qCheckBoxWarning->setCheckState(Qt::Unchecked);
    QVERIFY(cLogWidget->toPlainText().contains("log info test"));
    QVERIFY(!cLogWidget->toPlainText().contains("DEBUG", Qt::CaseInsensitive));
    QVERIFY(!cLogWidget->toPlainText().contains("ERROR", Qt::CaseInsensitive));
    QVERIFY(cLogWidget->toPlainText().contains("INFO", Qt::CaseInsensitive));
    QVERIFY(!cLogWidget->toPlainText().contains("WARNING", Qt::CaseInsensitive));

    qCheckBoxDebug->setCheckState(Qt::Unchecked);
    qCheckBoxError->setCheckState(Qt::Unchecked);
    qCheckBoxInfo->setCheckState(Qt::Unchecked);
    qCheckBoxWarning->setCheckState(Qt::Checked);
    QVERIFY(cLogWidget->toPlainText().contains("log warning test"));
    QVERIFY(!cLogWidget->toPlainText().contains("DEBUG", Qt::CaseInsensitive));
    QVERIFY(!cLogWidget->toPlainText().contains("ERROR", Qt::CaseInsensitive));
    QVERIFY(!cLogWidget->toPlainText().contains("INFO", Qt::CaseInsensitive));
    QVERIFY(cLogWidget->toPlainText().contains("WARNING", Qt::CaseInsensitive));

    CLogController::instance().getHistory().clearHistory();
}
