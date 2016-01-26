#ifndef IMAGEIO_H
#define IMAGEIO_H
#include <QImage>
#include <QUrl>

/*!
   \class CImageIo
 * \brief The CImageIo class
 * \author Laurenz Thiel
 *
 * Stellt die Möglichkeit bereit Bilder zu speichern und zu laden.
 */
class CImageIo
{
public:
    CImageIo();
    /*!
     * \brief Speichert ein Bild als PNG.
     * \param path Pfad zum Speicherort.
     * \param image Das zu speichernde Bild.
     * \return void
     */
    void save(QImage image, QUrl path);
    /*!
     * \brief Speichert ein Bild als PNG.
     * \param path Pfad zum Speicherort.
     * \return void
     */
    QImage load(QUrl path);
};

#endif // IMAGEIO_H
