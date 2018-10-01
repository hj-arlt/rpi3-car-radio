#ifndef AUDIOCTRL_H
#define AUDIOCTRL_H

#include <QObject>
#include <QString>
#include <QFile>
#include <QTextStream>
#include <QTimer>
#include <QAudio>
#include <QAudioDeviceInfo>
#include <QAudioOutput>
#include <QAudioInput>

#include "SystemCommand.h"

QT_BEGIN_NAMESPACE
class QAudioProbe;
QT_END_NAMESPACE

class AudioControl : public QObject
{
    Q_OBJECT

    static const int MAX_AUDIO_SOURCE = 2;
    enum Source
    {
        SOURCE_UNKNOWN = 0,
        SOURCE_CAPTURE = 1,
        SOURCE_PLAYBACK = 2,
    };

    typedef struct {
            bool init;
            int state;  // state in thread
            enum Source source;
            double lastvol[MAX_AUDIO_SOURCE];
            bool mute;
    } audio_str;

    audio_str audio;

    SystemCommand     m_command;

    QAudioOutput     *m_audioOutput;
    QIODevice        *m_audioOutputIODevice;
    QAudioDeviceInfo  m_audioOutputDevice;

    QAudioInput      *m_audioInput;
    QIODevice        *m_audioInputIODevice;
    QAudioDeviceInfo  m_audioInputDevice;
    QAudioFormat      m_format;

public:

    AudioControl();
    virtual ~AudioControl();

    bool isAudioAvailable() const;

    /* QML or public slot */
    Q_INVOKABLE void     mute(int chn);
    Q_INVOKABLE void     unmute(int chn);
    Q_INVOKABLE void     volume(int chn, double volume);
    Q_INVOKABLE int      setsourceChn(int chn);
    Q_INVOKABLE double   lastvolume(int chn);

signals:

    void changeVolume(int volume);
    void changeMuting(bool muting);

private slots:

    void audioInStateChanged(QAudio::State state);
    void audioOutStateChanged(QAudio::State state);
    void audioDataReady();
};

#endif // AUDIOCTRL_H
