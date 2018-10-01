#ifndef MEDIAPLAYER_H
#define MEDIAPLAYER_H

#include <QObject>
#include <QTime>
#include <QUrl>
#include <QTextStream>
#include <QFileInfo>
#include <QWidget>
#include <QMediaPlayer>
#include <QMediaService>
#include <QMediaPlaylist>
#include <QMediaMetaData>
#include <QVideoWidget>
#include <QMediaContent>
//#include <QVideoProbe>
//#include <QAudioProbe>

#include "SystemCommand.h"
#include "FileUtil.h"


QT_BEGIN_NAMESPACE
class QMediaPlayer;
class QModelIndex;
class QPushButton;
class QVideoWidget;
class PlaylistModel;
//class HistogramWidget;
QT_END_NAMESPACE

#define DEMO_USE_X_OVERLAY 1

class MediaPlayer : public QObject
{
    Q_OBJECT

public:

    MediaPlayer();
    virtual ~MediaPlayer();

    Q_INVOKABLE bool isPlayerAvailable();
    Q_INVOKABLE void addToPlaylist(const QList<QUrl> urls);

    Q_INVOKABLE int openAudio(const QString& source);
    Q_INVOKABLE int openVideo(const QString& source);
    Q_INVOKABLE void close();
    Q_INVOKABLE void setWindow(QWidget *winWidget);
    Q_INVOKABLE void setVideoWidget(QVideoWidget *videoWidget);

    Q_INVOKABLE void play();
    Q_INVOKABLE void pause();
    Q_INVOKABLE void stop();

signals:

    void error();
    void updateDuration(qint64 value, qint64 curr);
    void metaInfo(const QString& artist, const QString& title);
    void endOfStream();

private slots:

    void durationChanged(qint64 duration);
    void positionChanged(qint64 progress);
    void metaDataChanged();
    void seek(int seconds);
    void jump(int index);
    void playlistPositionChanged(int);
    void statusChanged(QMediaPlayer::MediaStatus status);
    void stateChanged(QMediaPlayer::State state);
    void bufferingProgress(int progress);
    void videoAvailableChanged(bool available);
    void previousClicked();
    void displayErrorMessage();

private:

    QMediaPlayer *player;
    QMediaPlaylist *playlist;
    PlaylistModel *playlistModel;
    QVideoWidget *videoWidget;
    QWidget *frameWidget;

    SystemCommand m_command;
    FileUtil m_fileutil;

    bool isVideo;
    QString trackInfo;
    QString statusInfo;
    qint64 max_duration;
    qint64 position;
    QMediaPlayer::State media_state;
    QMediaPlayer::MediaStatus media_status;


    void updateDurationInfo(qint64 currentInfo);
    void handleCursor(QMediaPlayer::MediaStatus status);

};

#endif // MEDIAPLAYER_H
