
#include <QDebug>
#include "MediaPlayer.h"

MediaPlayer::MediaPlayer()
    : m_command()
    , m_fileutil()
{
    qDebug("MediaPlayer::open");

    isVideo = false;
    frameWidget = 0;
    videoWidget = 0;

    player = new QMediaPlayer(this);
    // owned by PlaylistModel
    playlist = new QMediaPlaylist();
    player->setPlaylist(playlist);

    connect(player, SIGNAL(durationChanged(qint64)),    SLOT(durationChanged(qint64)));
    connect(player, SIGNAL(positionChanged(qint64)),    SLOT(positionChanged(qint64)));
    connect(player, SIGNAL(metaDataChanged()),          SLOT(metaDataChanged()));
    connect(player, SIGNAL(mediaStatusChanged(QMediaPlayer::MediaStatus)),
                this, SLOT(statusChanged(QMediaPlayer::MediaStatus)));
    connect(player, SIGNAL(bufferStatusChanged(int)),
                this, SLOT(bufferingProgress(int)));
    connect(player, SIGNAL(videoAvailableChanged(bool)),
                this, SLOT(videoAvailableChanged(bool)));
    connect(player, SIGNAL(error(QMediaPlayer::Error)),
                this, SLOT(displayErrorMessage()));
    connect(player, &QMediaPlayer::stateChanged, this, &MediaPlayer::stateChanged);
    connect(playlist, SIGNAL(currentIndexChanged(int)), SLOT(playlistPositionChanged(int)));

    if (!isPlayerAvailable()) {
        qDebug("The QMediaPlayer object does not have a valid service.\n"\
                "Please check the media service plugins are installed.");
    }
    metaDataChanged();

    qDebug("MediaPlayer::opened ok.");
}

MediaPlayer::~MediaPlayer()
{
    qDebug("MediaPlayer::closed\n");
}

bool MediaPlayer::isPlayerAvailable()
{
    return player->isAvailable();
}

int MediaPlayer::openAudio(const QString &source)
{
    isVideo = false;

    QList<QMediaContent> content;
    content.push_back(QUrl::fromLocalFile(source));

    qDebug() << "MediaPlayer::openAudio " << QUrl::fromLocalFile(source); // << " url " << content.first();

    playlist->addMedia(content);

    return 0;
}

int MediaPlayer::openVideo(const QString &source)
{
    QList<QMediaContent> content;
    content.push_back(QUrl::fromLocalFile(source));

    qDebug() << "MediaPlayer openVideo " << QUrl::fromLocalFile(source);

    playlist->addMedia(content);

    isVideo = true;

    return 0;
}

void MediaPlayer::setWindow(QWidget *winWidget)
{
    frameWidget = winWidget;
}

void MediaPlayer::setVideoWidget(QVideoWidget *videoWidget)
{
    this->videoWidget = videoWidget;
}

void MediaPlayer::play()
{
    qDebug() << "MediaPlayer play" << ((isVideo) ? "(video) " : "(audio) ");

    if (media_state == QMediaPlayer::PausedState) {
        player->play();
    }
    else if ((media_status != QMediaPlayer::LoadingMedia)
          && (media_state != QMediaPlayer::PlayingState)) {

        if (isVideo) {
/*
            if (! frameWidget) {
                qDebug() << "MediaPlayer ERR no frameWindow defined!!";
                return;
            }
            if (! videoWidget)
                videoWidget = new QVideoWidget(frameWidget);
*/
//            player->setVideoOutput(videoWidget);

            //connect(player, SIGNAL(stop()), videoWidget, SLOT(update()));

//            videoWidget->show();
        }

        playlist->setCurrentIndex(0);
        player->play();
    }
    else {
    }
}

void MediaPlayer::close()
{
    if (isVideo) {

    }
    else {
    }
    player->stop();
    playlist->clear();
}

static bool isPlaylist(const QUrl &url) // Check for ".m3u" playlists.
{
    if (!url.isLocalFile())
        return false;
    const QFileInfo fileInfo(url.toLocalFile());
    return fileInfo.exists() && !fileInfo.suffix().compare(QLatin1String("m3u"), Qt::CaseInsensitive);
}

/*
 * QUrl("file:///home/pi/car-radio/media/audio/Summertime Sadness.mp3")
 */
void MediaPlayer::addToPlaylist(const QList<QUrl> urls)
{
    foreach (const QUrl &url, urls) {
        if (isPlaylist(url)) {
            qDebug() << "addPlaylist: load " << url;
            playlist->load(url);
        } else {
            qDebug() << "addPlaylist: media " << url;
            playlist->addMedia(url);
        }
    }
}

void MediaPlayer::statusChanged(QMediaPlayer::MediaStatus status)
{
    qDebug() << "MediaPlayer status " << status;

    media_status = status;

    // handle status message
    switch (status) {
    case QMediaPlayer::UnknownMediaStatus:
    case QMediaPlayer::NoMedia:
    case QMediaPlayer::LoadedMedia:
    case QMediaPlayer::BufferingMedia:
    case QMediaPlayer::BufferedMedia:
        break;
    case QMediaPlayer::LoadingMedia:
        break;
    case QMediaPlayer::StalledMedia:
        break;
    case QMediaPlayer::EndOfMedia:
        break;
    case QMediaPlayer::InvalidMedia:
        displayErrorMessage();
        break;
    }
}

void MediaPlayer::stateChanged(QMediaPlayer::State state)
{
    qDebug() << "MediaPlayer state " << state;

    media_state = state;
}

void MediaPlayer::displayErrorMessage()
{
    qDebug() << "MediaPlayer error " << player->errorString();
}

void MediaPlayer::updateDurationInfo(qint64 currentInfo)
{
    QString tStr;
    if (currentInfo || max_duration) {
        QTime currentTime((currentInfo/3600)%60, (currentInfo/60)%60, currentInfo%60, (currentInfo*1000)%1000);
        QTime totalTime((max_duration/3600)%60, (max_duration/60)%60, max_duration%60, (max_duration*1000)%1000);
        QString format = "mm:ss";
        if (max_duration > 3600)
            format = "hh:mm:ss";
        tStr = currentTime.toString(format) + " / " + totalTime.toString(format);
    }
    qDebug() << "MediaPlayer time " << tStr;
}

void MediaPlayer::durationChanged(qint64 duration)
{
    max_duration = duration;
    qDebug() << "MediaPlayer max_duration " << max_duration;
}

void MediaPlayer::positionChanged(qint64 progress)
{
    position = progress / 1000;

    qDebug() << "MediaPlayer progress " << position << " / " << max_duration;

    updateDurationInfo(progress / 1000);

     emit updateDuration(max_duration, position);
}

void MediaPlayer::metaDataChanged()
{
    if (player->isMetaDataAvailable()) {
        QString trackArtist = player->metaData(QMediaMetaData::AlbumArtist).toString();
        if (trackArtist.length() > 30)
            trackArtist = trackArtist.left(30) + "...";

        QString trackTitle = player->metaData(QMediaMetaData::Title).toString();
        //int trackBitrate = player->metaData(QMediaMetaData::AudioBitRate).toInt();
        //QString coverImage = player->metaData(QMediaMetaData::CoverArtImage).toString();

        qDebug() << "MediaPlayer meta " << trackArtist << " " << trackTitle;

        emit metaInfo(trackArtist, trackTitle);
    }
}

void MediaPlayer::previousClicked()
{
    // Go to previous track if we are within the first 5 seconds of playback
    // Otherwise, seek to the beginning.
    if(player->position() <= 5000)
        playlist->previous();
    else
        player->setPosition(0);
}

void MediaPlayer::jump(int index)
{
        playlist->setCurrentIndex(index);
        player->play();
}

void MediaPlayer::playlistPositionChanged(int currentItem)
{
    qDebug() << "MediaPlayer position: " << currentItem;
//    clearHistogram();
//    playlistView->setCurrentIndex(playlistModel->index(currentItem, 0));
}

void MediaPlayer::seek(int seconds)
{
    qDebug() << "MediaPlayer seek: " << seconds;
    player->setPosition(seconds * 1000);
}

void MediaPlayer::handleCursor(QMediaPlayer::MediaStatus status)
{
#ifndef QT_NO_CURSOR
    if (status == QMediaPlayer::LoadingMedia ||
        status == QMediaPlayer::BufferingMedia ||
        status == QMediaPlayer::StalledMedia)
    {
        //        setCursor(QCursor(Qt::BusyCursor));
    }
//    else
//        unsetCursor();
#endif
}

void MediaPlayer::bufferingProgress(int progress)
{
    qDebug() << "MediaPlayer Buffering " << progress;
}

void MediaPlayer::videoAvailableChanged(bool available)
{
    qDebug() << "MediaPlayer video avail: " << available;
#if 0
    if (!available) {
        disconnect(fullScreenButton, SIGNAL(clicked(bool)),
                    videoWidget, SLOT(setFullScreen(bool)));
        disconnect(videoWidget, SIGNAL(fullScreenChanged(bool)),
                fullScreenButton, SLOT(setChecked(bool)));
        videoWidget->setFullScreen(false);
    } else {
        connect(fullScreenButton, SIGNAL(clicked(bool)),
                videoWidget, SLOT(setFullScreen(bool)));
        connect(videoWidget, SIGNAL(fullScreenChanged(bool)),
                fullScreenButton, SLOT(setChecked(bool)));

        if (fullScreenButton->isChecked())
            videoWidget->setFullScreen(true);
    }
    colorButton->setEnabled(available);
#endif
}

void MediaPlayer::pause()
{
    qDebug() << "MediaPlayer pause" << ((isVideo) ? "(video)" : "(audio)");

    player->pause();
}

void MediaPlayer::stop()
{
    qDebug() << "MediaPlayer stop" << ((isVideo) ? "(video)" : "(audio)");

    if (isVideo)
    {
    }
    else {
    }
    player->stop();
    playlist->clear();
}
