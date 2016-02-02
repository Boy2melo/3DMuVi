#include <QtTest>

#include <io/CTextIo.h>

/*!
\brief Test for CTextIo.
\author Laurenz Thiel

This class is a unit test for CTextIo.
*/
class CTestCTextIo : public QObject
{
  Q_OBJECT

private slots:
  void initTestCase();
  void cleanupTestCase();

  /*!
  \brief Test for save.

  This is a test for save with valid input data.
   */
  void save();

  /*!
  \brief Test for load.

  This is a test for load with valid input data.
   */
  void load();

private:
  CTextIo textIo;
  QString testText;
  QUrl testTextPath;
  QUrl testTextPathToSave;
};

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

QTEST_MAIN(CTestCTextIo)
#include  "CTestCTextIo.moc"
