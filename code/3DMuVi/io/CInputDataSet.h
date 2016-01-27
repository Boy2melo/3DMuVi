#ifndef INPUTDATASET_H
#define INPUTDATASET_H
#include <QUrl>
#include <QImage>
#include <QListWidgetItem>
#include <tuple>
#include <QIcon>
#include <QDir>
#include <QStringList>
#include <tuple>
#include <iostream>
#include <QBitmap>

/*!
   \class CInputDataSet
 * \brief The CInputDataSet class
 * \author Laurenz Thiel
 *
 * Stellt Bilder aus einem Ordner für den Workflow als Eingabedaten bereit.
 */
class CInputDataSet
{
public:
    /*!
     * \brief Ist der Konstruktor der Klasse. Lädt alle Eingabebilder, versieht diese mit einer Id und generiert zu jedem Bild ein Thumbnail.
     * \param path Pfad zum Ordner in dem die Eingabebilder liegen.
     * \return void
     */
    CInputDataSet(QUrl path);
    /*!
     * \brief Gibt die vom Konstruktor erzeugten Daten als Tripel zurück.
     * \return Es wird ein Pointer auf einen Vector zurückgegeben. Jeder Vektor enthält ein Triple. Die erste Komponente
     * ist die Id, die zweite ein Bild in originaler Auflösung und die dritte ein Objekt
     * vom Typ QListWidgetItem. Darin enthalten ist ein Thumbnail des Bildes welches
     * auf der GUI durch ein QListWidget angezeigt werden kann.
     */
    std::vector<std::tuple<u_int32_t, QImage, QListWidgetItem>>* getInputImages();
private:
    std::vector<std::tuple<u_int32_t, QImage, QListWidgetItem>> inputData;
};

#endif // INPUTDATASET_H
