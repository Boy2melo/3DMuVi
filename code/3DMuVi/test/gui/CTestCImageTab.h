#pragma once
#include <QTest>
#include <QTemporaryDir>

class CTestCImageTab : public QObject {
    Q_OBJECT;
    private slots:
    /**
    \brief Teste das Laden von zwei Bildern
    */
    void test2Images();
    /**
    \brief Teste das Laden von 0 Bildern
    */
    void test0Images();
    /**
    \brief Teste das Laden von 500 Bildern
    */
    void test500Images();

    /**
    \brief Teste das Laden von gemischten Dateitypen
    */
    void testTypes();

private:
    void createImages(QDir path, uint images, const char ext[]);

    void validateView(QDir dir, int expected);
};

