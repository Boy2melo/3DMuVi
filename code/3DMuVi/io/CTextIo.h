#ifndef CTEXTIO_H
#define CTEXTIO_H
#include <QUrl>
#include <QString>
#include <QFile>
#include <QTextStream>

/*!
   \class CTextIo
 * \brief The CTextIo class
 * \author Laurenz Thiel
 *
 * Stellt die Möglichkeit bereit Texte zu speichern und zu laden.
 */
class CTextIo
{
public:
    CTextIo();
    /*!
     * \brief Speichert einen Text in einer in path spezifizierten Datei.
     * \param path Pfad zum Speicherort.
     * \param text Inhalt der zu speichernden Datei.
     * \return void
     */
    void save(QUrl path,QString text);
    /*!
     * \brief Lädt den Text aus einer in path spezifizierten Datei.
     * \param path Pfad zum Ladeort.
     * \return Text aus der Datei.
     */
    QString load(QUrl path);
};

#endif // CTEXTIO_H
