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
private:
    CTextIo();
public:
    /*!
     * \brief Speichert einen Text in einer in path spezifizierten Datei.
     * \return void
     */
    void save(QUrl path,QString text);
    /*!
     * \brief Lädt den Text aus einer in path spezifizierten Datei.
     * \return Text aus der Datei.
     */
    QString load(QUrl path);
};

#endif // CTEXTIO_H
