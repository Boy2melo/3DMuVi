#ifndef INPUTDATASET_H
#define INPUTDATASET_H
#include <QUrl>
#include <QImage>
#include <gui/CImagePreviewItem.h>
#include <tuple>
#include <QIcon>
#include <QDir>
#include <QStringList>
#include <tuple>
#include <iostream>
#include <QBitmap>
#include <workflow/workflow/datapackets/CDataFeature.h>

/*!
   \class CInputDataSet
 * \brief The CInputDataSet class
 * \author Laurenz Thiel
 *
 * Stellt Bilder aus einem Ordner für den Workflow als Eingabedaten bereit.
 */
class CInputDataSet : public IDataPacket {
public:
    /*!
     * \brief Ist der Konstruktor der Klasse. Lädt alle Eingabebilder, versieht diese mit einer Id und generiert zu jedem Bild ein Thumbnail.
     * \param path Pfad zum Ordner in dem die Eingabebilder liegen.
     * \return void
     */
    CInputDataSet(QUrl path);

    CInputDataSet() {}

    /*!
     * \brief Gibt die vom Konstruktor erzeugten Daten als Tripel zurück.
     * \return Es wird ein Pointer auf einen Vector zurückgegeben. Jeder Vektor enthält ein Triple. Die erste Komponente
     * ist die Id, die zweite ein Bild in originaler Auflösung und die dritte ein Objekt
     * vom Typ CImagePreviewItem. Darin enthalten ist ein Thumbnail des Bildes welches
     * auf der GUI durch ein QListWidget angezeigt werden kann.
     */
    std::vector<std::tuple<uint32_t, QImage, CImagePreviewItem>>* getInputImages();

    QString getDataType() const override;
    AStreamProvider* getStreamProvider() override;
    void serialize(AStreamProvider* stream) override {}
    void deserialize(AStreamProvider* stream) override {}
private:
    std::vector<std::tuple<uint32_t, QImage, CImagePreviewItem>> inputData;
};

#endif // INPUTDATASET_H
