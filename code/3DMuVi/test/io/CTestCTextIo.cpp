#include <QtTest>

#include "CTestCTextIo.h"

void CTestCTextIo::initTestCase()
{
    testTextPathToSave.setUrl("testSaveText.txt");
    testTextPath.setUrl(":/data/io/data/testText.txt");
    testText = "The quick brown fox jumps over the lazy dog.\n" \
               "This is a pangram.\n";
}

void CTestCTextIo::save()
{
    textIo.save(testTextPathToSave.path(),testText);

    QString result("");
    QFile file(testTextPathToSave.path());

    if (file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QTextStream in(&file);
        while(!in.atEnd())
        {
            QString line = in.readLine();
            line.append("\n");
            result.append(line);
        }
    }
  QCOMPARE(result, testText);
}

void CTestCTextIo::load()
{
  QCOMPARE(textIo.load(testTextPath) , testText);
}

void CTestCTextIo::cleanupTestCase()
{
    QFile::remove(testTextPathToSave.path());
}
