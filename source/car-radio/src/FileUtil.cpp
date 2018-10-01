
#include <QDebug>
#include <QCoreApplication>

#include <dirent.h>
#include "id3v2lib.h"
#include "FileUtil.h"

QString FileUtil::getAudioBase()
{
    QString mediaBase = QCoreApplication::applicationDirPath();

    mediaBase += "/media/audio";
    return mediaBase;
}

QString FileUtil::getAudioURIBase()
{
    return ("file://" + getAudioBase());
}

QString FileUtil::getVideoBase()
{
    QString mediaBase = QCoreApplication::applicationDirPath();

    mediaBase += "/media/video";
    return mediaBase;
}

QString FileUtil::getVideoURIBase()
{
    return ("file://" + getVideoBase());
}

QString FileUtil::getRadioBase()
{
    QString radioBase = QCoreApplication::applicationDirPath();

    radioBase += "/media/radio";
    return radioBase;
}

QString FileUtil::getRadioURIBase()
{
    return ("file://" + getRadioBase());
}

QString FileUtil::getSystemBase()
{
    QString mediaBase = QCoreApplication::applicationDirPath();

    mediaBase += "/system";
    return mediaBase;
}

QString FileUtil::getSystemURIBase()
{
    return ("file://" + getSystemBase());
}

QString FileUtil::getLogoBase()
{
    QString mediaBase = QCoreApplication::applicationDirPath();

    mediaBase += "/qml/images/logos";
    return mediaBase;
}

int FileUtil::getFileLines(QString fileName)
{
    QString line;

    QFile inputFile(fileName);
    if (!inputFile.open(QIODevice::ReadOnly)) {
        return 0;
    }
    QTextStream in(&inputFile);
    int i = 0;
    while(!in.atEnd()) {
        line = in.readLine();
        if ((line.contains("FM")) || (line.contains("AM")) || (line.contains("DAB")) || (line.contains("INET")))
            i++;
    }
    inputFile.close();
    return i;
}

QString FileUtil::getFileLine(QString fileName, int linenumber)
{
    QString line;
    QFile inputFile(fileName);
    if (!inputFile.open(QIODevice::ReadOnly)) {
        return "";
    }
    QTextStream in(&inputFile);
    int i = 0;
    while(!in.atEnd()) {
        line = in.readLine();
        if ((line.contains("FM")) || (line.contains("AM")) || (line.contains("DAB")) || (line.contains("INET")))
        {
            if (i == linenumber)
                break;
        }
        i++;
    }
    inputFile.close();
    if (i == linenumber) {
//     qDebug() << line;
       return line;
    }
    return "empty";
}

QString FileUtil::getLineItem(QString line, int item)
{
    QString str, sub, next;
    int i, pos;
    sub = line;
    for (i = 0; i<item; i++)
    {
        pos = sub.indexOf(",", 0, Qt::CaseInsensitive);
        str = sub.left(pos);
        next = sub.right(sub.length() - pos - 1); // next
        sub = next;
//      qDebug() << "item " << i << " pos " << pos << " " << str << " " << next;
    }
    return str;
}

QString FileUtil::getFolderEntry(QString folder, int item)
{
    int            cnt;
    DIR           *d;
    struct dirent *dir;
    QString        file;
//    qDebug() << "Open path: " << folder << " " << (char*)(folder.toStdString().c_str());
    d = opendir((char*)folder.toStdString().c_str());
    if (d == NULL)
    {
        qDebug() << "ERR: open path " << folder;
        return "";
    }
    cnt = 0;
    while ((dir = readdir(d)) != NULL)
    {
//        qDebug() << "entry: " << cnt << " " << item << " " << dir->d_name;
        if (dir->d_name[0] != '.') {
//          qDebug() << "entry: " << cnt << " " << item << " " << dir->d_name;
            file = dir->d_name;
            if (cnt == item) {
                closedir(d);
                return file;
            }
            cnt++;
        }
    }
    closedir(d);
    return "";
}

int FileUtil::getMp3Tag(QString fileName)
{
    QString str;
    char txt[64];
    ID3v2_tag* tag = load_tag(fileName.toStdString().c_str()); // Load the full tag from the file
    if(tag == NULL)
    {
        tag = new_tag();
    }
    // Load the fields from the tag
    ID3v2_frame* artist_frame = tag_get_artist(tag); // Get the full artist frame
    // We need to parse the frame content to make readable
    ID3v2_frame_text_content* artist_content = parse_text_frame_content(artist_frame);
    strncpy(txt, artist_content->data, 64);
    txt[artist_content->size] = 0;
    str = QString(txt);
//    qDebug() << "ARTIST: " << str; // Show the artist info
    mp3.artist = str;

    ID3v2_frame* title_frame = tag_get_title(tag);
    ID3v2_frame_text_content* title_content = parse_text_frame_content(title_frame);
    strncpy(txt, title_content->data, 64);
    txt[title_content->size] = 0;
    str = QString(txt);
//    qDebug() << "TITLE: " << str;
    mp3.title = str;

    ID3v2_frame* album_frame = tag_get_album(tag);
    ID3v2_frame_text_content* album_content = parse_text_frame_content(album_frame);
    strncpy(txt, album_content->data, 64);
    txt[album_content->size] = 0;
    str = QString(txt);
//    qDebug() << "ALBUM: " << str; // Show the album info
    mp3.album = str;

    return 0;
}

QString FileUtil::getMp3Artist()
{
    return mp3.artist;
}

QString FileUtil::getMp3Album()
{
    return mp3.album;
}
QString FileUtil::getMp3Title()
{
    return mp3.title;
}

/*
 * ffmpeg -i Video_Games.mp3 -an -vcodec copy cover.jpg
 */
int FileUtil::getMp3Cover(QString fileName, QString pictureName)
{
    int i;
    unsigned char *pCover;
    FILE *fp;
    ID3v2_tag* tag = load_tag(fileName.toStdString().c_str()); // Load the full tag from the file
    if(tag == NULL)
    {
        tag = new_tag();
    }
    ID3v2_frame* cover_frame = tag_get_album_cover(tag);
    ID3v2_frame_text_content* cover_content = parse_text_frame_content(cover_frame);
    pCover = (unsigned char*)cover_content->data;
    for (i=0; i<cover_content->size; i++) {
        if ((pCover[i] == 0xFF) && (pCover[i+1] == 0xD8) && (pCover[i+2] == 0xFF) && (pCover[i+3] == 0xE0))
            break;
    }
    pCover += i;
//    qDebug() << "cover size " << cover_content->size;

//    QString filename = getSystemBase() + "/" + "tmppic.jpg";
    fp = fopen(pictureName.toStdString().c_str(), "w+");
    if (fp) {
        fwrite(pCover, 1, cover_content->size - i, fp);
        fclose(fp);
        return 0;
    }
    else {
        qDebug() << "Error: open.." << pictureName;
    }
    return -1;
}
