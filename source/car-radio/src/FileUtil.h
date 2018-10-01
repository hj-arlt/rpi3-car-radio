#ifndef FILEUTIL_H
#define FILEUTIL_H

#include <QObject>
#include <QString>
#include <QFile>
#include <QDataStream>
#include <QTextStream>
#include <QStringList>

class FileUtil : public QObject
{
    Q_OBJECT

public:

    Q_INVOKABLE QString getAudioBase();
    Q_INVOKABLE QString getAudioURIBase();
    Q_INVOKABLE QString getVideoBase();
    Q_INVOKABLE QString getVideoURIBase();
    Q_INVOKABLE QString getRadioBase();
    Q_INVOKABLE QString getRadioURIBase();
    Q_INVOKABLE QString getSystemBase();
    Q_INVOKABLE QString getSystemURIBase();
    Q_INVOKABLE QString getLogoBase();
    Q_INVOKABLE int getFileLines(QString fileName);
    Q_INVOKABLE QString getFileLine(QString fileName, int line);
    Q_INVOKABLE QString getLineItem(QString line, int item);
    Q_INVOKABLE QString getFolderEntry(QString dir, int item);
    Q_INVOKABLE int getMp3Tag(QString fileName);
    Q_INVOKABLE int getMp3Cover(QString fileName, QString pictureName);
    Q_INVOKABLE QString getMp3Artist();
    Q_INVOKABLE QString getMp3Album();
    Q_INVOKABLE QString getMp3Title();

private:

    struct mp3tag
    {
        QString artist;
        QString title;
        QString album;
        char *cover;
    };

    struct mp3tag mp3;
};

#endif // FILEUTIL_H
