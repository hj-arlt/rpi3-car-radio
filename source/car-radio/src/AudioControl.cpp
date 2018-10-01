/*
 *  audioctrl.cpp
 *
 *  control of audio parts without wigdets
 *  connected to qml interface
 *
 *  Created on: 30.03.2018
 *      Author: hj.arlt@online.de
 */
#include <QDebug>

#include "AudioControl.h"

/* ********************************************************************
 * class audioctrl
 * - control alsa device
 ********************************************************************** */

AudioControl::AudioControl()
    : m_command()
{
    QList<QAudioDeviceInfo> availableDevices;
    int i;
    const QString defaultOutput = "analog-stereo";
    const QString defaultInput  = "plughw:";

    qDebug() << "AudioControl open..";

    audio.source     = SOURCE_UNKNOWN;
    audio.lastvol[0] = 0.05;  // radio
    audio.lastvol[1] = 0.05;  // player
    audio.mute = false;

#ifdef PULSE_AUDIO
    m_command.invoke("arecord -D plughw:0,0 -f dat | aplay -f dat &");
    m_command.invoke("amixer set Master 100%");
#endif
    /* a little volume level over */
    m_command.invoke("pactl set-sink-volume 0 100%");

    // Select the output device that I want
    availableDevices = QAudioDeviceInfo::availableDevices(QAudio::AudioOutput);
    qDebug() << "Output devices: available " << availableDevices.size();
    for(i=0;i<availableDevices.size();i++){
        //qDebug() << availableDevices.at(i).deviceName();
        if(availableDevices.at(i).deviceName().contains(defaultOutput)) {
            m_audioOutputDevice = availableDevices.at(i);
            break;
        }
    }
    if (i>=availableDevices.size())
        m_audioOutputDevice = QAudioDeviceInfo::defaultOutputDevice();

    // Select the input device that I want
    availableDevices = QAudioDeviceInfo::availableDevices(QAudio::AudioInput);
    qDebug() << "Input devices: available " << availableDevices.size();
    for(i=0;i<availableDevices.size();i++){
        //qDebug() << availableDevices.at(i).deviceName();
        if(availableDevices.at(i).deviceName().contains(defaultInput)) {
            m_audioInputDevice = availableDevices.at(i);
            break;
        }
    }
    if (i>=availableDevices.size())
        m_audioInputDevice = QAudioDeviceInfo::defaultInputDevice();

    qDebug() << "AudioControl: Selected Input device  :" << m_audioInputDevice.deviceName();
    qDebug() << "AudioControl: Selected Output device :" << m_audioOutputDevice.deviceName();

    m_format.setChannelCount(2);
    m_format.setCodec("audio/pcm");
    m_format.setSampleSize(16);
    m_format.setSampleRate(48000);

    // Check format is OK
    if(!m_audioInputDevice.isFormatSupported(m_format)){
        qWarning() << "Default format not supported, trying to use the nearest.";
        m_format = m_audioInputDevice.nearestFormat(m_format);
    }
    qDebug() << "\tCodec:" << m_format.codec();
    qDebug() << "\tChannel count:" << m_format.channelCount();
    qDebug() << "\tSample size:" << m_format.sampleSize();
    qDebug() << "\tSample rate:" << m_format.sampleRate();

    if(m_format.isValid()){
        // Initialisation du micro
        m_audioInput = new QAudioInput(m_audioInputDevice,m_format,this);
        m_audioInput->setNotifyInterval(100); // ms
        connect(m_audioInput,SIGNAL(stateChanged(QAudio::State)),this,SLOT(audioInStateChanged(QAudio::State)));

        m_audioInputIODevice = m_audioInput->start();
        if (m_audioInput->error() != QAudio::NoError) {
            qCritical() << "ERR: audioIn " << m_audioInput->error();
        }
        connect(m_audioInputIODevice,SIGNAL(readyRead()),this,SLOT(audioDataReady()));
    }
    else {
        qCritical() << "Problem audio format";
    }

    /* output with same format like input */

    m_audioOutput = new QAudioOutput(m_audioOutputDevice, m_format, this);
    if (m_audioOutput == NULL) {
        qCritical() << "Problem audio output";
    }
    connect(m_audioOutput,SIGNAL(stateChanged(QAudio::State)),this,SLOT(audioOutStateChanged(QAudio::State)));

    m_audioOutput->setBufferSize(96000);

    setsourceChn(0);

    audio.init = true;
}

AudioControl::~AudioControl()
{
    qDebug() << "audictrl::closed!";
}

bool AudioControl::isAudioAvailable() const
{
    return audio.init;
}

void  AudioControl::mute(int chn)
{
    if (audio.init && (chn > MAX_AUDIO_SOURCE-1))
            return;
    qDebug() << "audioctrl chn " << chn << " mute";

#ifdef PULSE_AUDIO
    m_command.invoke("amixer set Master mute");
#else
    m_audioOutput->setVolume(0);
#endif
    audio.mute = true;
}

void  AudioControl::unmute(int chn)
{
    if (audio.init && (chn > MAX_AUDIO_SOURCE-1))
            return;
    qDebug() << "audioctrl chn " << chn << " unmute";

#ifdef PULSE_AUDIO
    m_command.invoke("amixer set Master unmute");
#else
    m_audioOutput->setVolume(audio.lastvol[chn]);
#endif
    audio.mute = false;
}

/* volume 0.00 .. 1.00 */
void  AudioControl::volume(int chn, double volume)
{
    if (audio.init && (chn > MAX_AUDIO_SOURCE-1))
            return;

    qDebug() << "audioctrl chn " << chn << " volume" << volume;
    audio.lastvol[chn] = volume;

#ifdef PULSE_AUDIO
    int vol = 250;
    QString cmd = "pactl set-sink-volume 0 ";
    vol = volume * vol;
    cmd.append(QString::number(vol));
    cmd.append("%");
    m_command.invoke(cmd);
#else
    m_audioOutput->setVolume(audio.lastvol[chn]);
#endif
}

/* volume 0.00 .. 1.00 */
double AudioControl::lastvolume(int chn)
{
    if (audio.init && (chn > MAX_AUDIO_SOURCE-1))
            return 0;

    return (audio.lastvol[chn]);
}

/*
 * special switch:
 * Mediaplayer is played from system (file)..
 * Tuner is played with alsa loop capture to playback..
 */
int  AudioControl::setsourceChn(int chn)
{
    if (audio.init && (chn > MAX_AUDIO_SOURCE-1))
            return -1;
    qDebug() << "audioctrl source chn" << ((chn==0)?"radio":"music");


    /* radio */
    if ((chn == SOURCE_CAPTURE-1) && (audio.source != SOURCE_CAPTURE))
    {
        audio.source = SOURCE_CAPTURE;
#ifdef PULSE_AUDIO
        m_command.invoke("arecord -D plughw:0,0 -f dat | aplay -f dat &");
#else
        m_audioOutput->stop();
        m_audioOutput->reset();
        m_audioOutput->start(m_audioInputIODevice);
        if (m_audioOutput->error() != QAudio::NoError) {
            qDebug() << "ERR: src 0 audioOut " << m_audioOutput->error();
            return -1;
        }
        if (! audio.mute)
            m_audioOutput->setVolume(audio.lastvol[0]);
#endif
    }
    /* musik */
    else if (chn == SOURCE_PLAYBACK-1) //&& (audio.source != SOURCE_PLAYBACK))
    {
        audio.source = SOURCE_PLAYBACK;
#ifdef PULSE_AUDIO
        m_command.invoke("pkill aplay");
        m_command.invoke("pkill arecord");
#elif 0 //se
        m_audioOutput->stop();
        m_audioOutput->reset();
        m_audioOutput->start();
        if (m_audioOutput->error() != QAudio::NoError) {
            qDebug() << "ERR: audioOut " << m_audioOutput->error();
            return -1;
        }
        if (! audio.mute)
            m_audioOutput->setVolume(audio.lastvol[1]);
#endif
        m_audioOutput->setVolume(0);
    }
    return 0;
}

void AudioControl::audioOutStateChanged(QAudio::State state)
{
    qDebug() << "audictrl:: out state " << state;

    switch (state)
    {
    case QAudio::IdleState:
        break;

    case QAudio::ActiveState:
        qDebug() << "AudioControl buffer " <<  m_audioOutput->bufferSize();
        break;

    case QAudio::StoppedState:
        // Stopped for other reasons
        if (m_audioOutput->error() != QAudio::NoError) {
            // Error handling
            qDebug() << "ERR: audictrl:: out state " << state;
        }
        break;

    default:
        // ... other cases as appropriate
        break;
    }
}

void AudioControl::audioInStateChanged(QAudio::State state)
{
    qDebug() << "audictrl:: in state " << state;
    switch (state)
    {
    case QAudio::IdleState:
        break;

    case QAudio::ActiveState:
        break;

    case QAudio::StoppedState:
        // Stopped for other reasons
        if (m_audioOutput->error() != QAudio::NoError) {
            // Error handling
            qDebug() << "ERR: audictrl:: in state " << state;
        }
        break;

    default:
        // ... other cases as appropriate
        break;
    }
}

void AudioControl::audioDataReady()
{
    //qDebug() << "audictrl:: data ready";
}
