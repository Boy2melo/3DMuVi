#include "CTextIo.h"

void CTextIo::save(QUrl path, QString  text) {
    QFile file(path.toString());
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
        return;

    QTextStream out(&file);
    out << text;
}

QString CTextIo::load(QUrl path) {
    QString result("");
    QFile file(path.path());

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return result;

    QTextStream in(&file);
    while (!in.atEnd()) {
        QString line = in.readLine();
        line.append("\n");
        result.append(line);
    }
    return result;
}
