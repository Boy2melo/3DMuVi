#ifndef CTESTTABLOG_H
#define CTESTTABLOG_H

#include <QObject>

class CTestTabLog : public QObject
{
    Q_OBJECT
public:
    explicit CTestTabLog(QObject *parent = 0);

signals:

public slots:

private slots:
    void test();
};

#endif // CTESTTABLOG_H
