/**************************************************************************** 
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
** Modified 22 January 2018 by RoBoardGod
****************************************************************************/

#include <QtWidgets>
#include <QtNetwork>
#include <QProcess>
#include <QDebug>

#include "mainwindow.h"
#include "tcp86Client.h"

#define cag(a) parent->checkAndGetValue(a)
#define cagInt(a) parent->checkAndGetValueInt(a)


TCP86Client::TCP86Client(MainWindow *p, int t_port,QWidget *parent)
    : QDialog(parent), parent(p)
{
    target_port = t_port;
    tcpSocket = new QTcpSocket(this);
    connect(tcpSocket, &QIODevice::readyRead, this, &TCP86Client::readFortune);
    connect(tcpSocket, QOverload<QAbstractSocket::SocketError>::of(&QAbstractSocket::error),
            this, &TCP86Client::displayError);
    status_timer = new QTimer(this);
    connect(status_timer, SIGNAL(timeout()), this, SLOT(status_update()));
}

void TCP86Client::set_Port(int t_port){
    target_port = t_port;
}

void TCP86Client::sessionOpened(QStringList parameterList)
{
    Python_process = new QProcess();
    Python_process->setWorkingDirectory(QDir::currentPath() + "/Helper");
    Python_process->start("python", parameterList);
    if (Python_process->error() == QProcess::FailedToStart)
    {
        return;
    }
    Python_process->waitForStarted(cagInt("PythonWaitStart"));

    tcpSocket->abort();
    tcpSocket->connectToHost(QHostAddress::LocalHost,target_port);
    already = true;

    status_timer->start(cagInt("TCPStatusTimer"));
}

void TCP86Client::readFortune()
{

    QByteArray buffer;
    unsigned char *ch;

    if(tcpSocket->bytesAvailable()){
        buffer = tcpSocket->readAll();
        ch = (unsigned char*)buffer.data();
        if(ch[0] == 0xFE && ch[1] == 0xFE){
            switch (ch[2]){
                case 0xC1:
                {
                    if(ch[3] >= finish_status){
                        status_timer->stop();
                        sendFortune(0x90);
                        status_timer->start(500);
                        qDebug() << "finish";
                    }
                    emit helper_status(ch[3]);
                    break;
                }
                case 0xA1:
                {
                    int _S2P = (0xFF00 & ((unsigned short)ch[3] << 8)) | (ch[4] & 0xFF);
                    int _P2B = (0xFF00 & ((unsigned short)ch[5] << 8)) | (ch[6] & 0xFF);
                    int _B2P = (0xFF00 & ((unsigned short)ch[7] << 8)) | (ch[8] & 0xFF);
                    int _P2S = (0xFF00 & ((unsigned short)ch[9] << 8)) | (ch[10] & 0xFF);
                    emit helper_speed(_S2P, _P2B, _B2P, _P2S);
                    break;
                }
                case 0x91:
                {
                    QString str = QString::number(ch[3]) + "." +
                            QString::number(ch[4]) + "." +
                            QString::number(ch[5]) + "." +
                            QString::number(ch[6]);
                    emit helper_IP( &str );
                    break;
                }
                case 0xB0:
                {
                    emit helper_error(ch[3]);
                    break;
                }
            }
        }

    }

}
void TCP86Client::displayError(QAbstractSocket::SocketError socketError)
{
    switch (socketError) {
    case QAbstractSocket::RemoteHostClosedError:
        break;
    case QAbstractSocket::HostNotFoundError:
        QMessageBox::information(this, tr("Fortune Client"),
                                 tr("The host was not found. Please check the "
                                    "host name and port settings."));
        break;
    case QAbstractSocket::ConnectionRefusedError:
        QMessageBox::information(this, tr("Fortune Client"),
                                 tr("The connection was refused by the peer. "
                                    "Make sure the fortune server is running, "
                                    "and check that the host name and port "
                                    "settings are correct."));
        break;
    default:
        QMessageBox::information(this, tr("Fortune Client"),
                                 tr("The following error occurred: %1.")
                                 .arg(tcpSocket->errorString()));
    }

}
void TCP86Client::sendFortune(unsigned char cmd)
{
    if(tcpSocket->state() == QTcpSocket::ConnectedState){
        QByteArray ba;
        ba.resize(3);
        ba[0] = 0xFE;
        ba[1] = 0xFE;
        ba[2] = cmd;
        tcpSocket->write(ba);
        tcpSocket->flush();
    }
}

void TCP86Client::status_update()
{
    if(connect_status < finish_status)
        sendFortune(0xC0);
    sendFortune(0xA0);
}
void TCP86Client::TCPdisconnect()
{
    status_timer->stop();
    if(connect_status != 0){
        sendFortune(0x80);
        Python_process->waitForFinished(2000);
    }
    if(already){
        tcpSocket->disconnectFromHost();
        already = false;
    }
    connect_status = -1;
    connect_error = -1;
    Python_process->close();
}

/*
 * Show 86Duino Status
 */
void TCP86Client::status_Show(int status, QString *&str1, QString *&str2)
{
    switch(status){
        case 0:
            str1 = new QString(cag("str1_status0"));str2 = new QString(cag("str2_status0"));
            break;
        case 1:
            str1 = new QString(cag("str1_status1"));str2 = new QString(cag("str2_status1"));
            break;
        case 2:
            str1 = new QString(cag("str1_status2"));str2 = new QString(cag("str2_status2"));
            break;
        case 3:
            str1 = new QString(cag("str1_status3"));str2 = new QString(cag("str2_status3"));
            break;
        case 4:
            str1 = new QString(cag("str1_status4"));str2 = new QString(cag("str2_status4"));
            break;
        case 5:
            str1 = new QString(cag("str1_status5"));str2 = new QString(cag("str2_status5"));
            break;
        case 6:
            str1 = new QString(cag("str1_status6"));str2 = new QString(cag("str2_status6"));
            break;
        case 7:
            str1 = new QString(cag("str1_status7"));str2 = new QString(cag("str2_status7"));
            break;
        case 8:
            str1 = new QString(cag("str1_status8"));str2 = new QString(cag("str2_status8"));
            break;
        case 9:
            str1 = new QString(cag("str1_status9"));str2 = new QString(cag("str2_status9"));
            break;
        case 10:
            str1 = new QString(cag("str1_status10"));str2 = new QString(cag("str2_status10"));
            break;
        default:
            str1 = nullptr;
            str2 = nullptr;
            break;
    }
    /*
    switch(status){
        case 0:     return "86Scratch超人：86菌～你在哪裡～？";
        case 1:     return "86Scratch超人：原來你在這裡阿！";
        case 2:     return "86Scratch超人：原來你在這裡阿！快點準備好！我們要出發了！";
        case 3:     return "86Scratch超人：86菌！";
        case 4:     return "86Scratch超人：86菌！裝備有沒有充足呢？";
        case 5:     return "86Scratch超人：86菌！裝備有沒有充足呢？我們來檢察看看吧！";
        case 6:     return "86Scratch超人：86菌！";
        case 7:     return "86Scratch超人：86菌！這些裝備有哪些種類阿？";
        case 8:     return "86Scratch超人：86菌！這些裝備有哪些種類阿？我們來好好清點一下吧！";
        case 9:     return "86Scratch超人：檢查完成啦！讓我們開啟Scratch傳送門開始今天的冒險吧！";
        case 10:    return "86Scratch超人：讓我們進入Scratch的世界，捍衛和平吧！";
        default:    return "";
    }
    */
}

/*
 * Show 86Duino Error Code
 */
void TCP86Client::errorcode_Show(int errorcode, QString *&str1, QString *&str2)
{
    switch(errorcode){
        case 1:
            str1 = new QString(cag("str1_error1"));
            str2 = new QString(cag("str2_error1"));
            break;
        case 2:
            str1 = new QString(cag("str1_error2"));
            str2 = new QString(cag("str2_error2"));
            break;
        case 3:
            str1 = new QString(cag("str1_error3"));
            str2 = new QString(cag("str2_error3"));
            break;
        case 4:
            str1 = new QString(cag("str1_error4"));
            str2 = new QString(cag("str2_error4"));
            break;
        case 5:
            str1 = new QString(cag("str1_error5"));
            str2 = new QString(cag("str2_error5"));
            break;
        case 6:
            str1 = new QString(cag("str1_error6"));
            str2 = new QString(cag("str2_error6"));
            connect_status = 9;
            break;
        case 99:
            str1 = new QString(cag("str1_error99"));
            str2 = new QString(cag("str2_error99"));
            break;
        default:
            str1 = nullptr;
            str2 = nullptr;
            break;
    }
    /*
    switch(errorcode){
        case 1:     return "86Scratch超人：找不到86菌阿！86菌跑到哪裡去了呢？";
        case 2:     return "86Scratch超人：86菌好像沒聽到我說話呢？";
        case 3:     return "86Scratch超人：86菌！裝備有沒有充足呢？咦？怎麼不理我了呢？";
        case 4:     return "86Scratch超人：86菌！這些裝備有哪些種類阿？咦？怎麼不理我了呢？";
        case 5:     return "86Scratch超人：傳送門...故障了！？";
        case 99:    return "86Scratch超人：已經沒辦法和86菌通訊了。";
        default:    return "";
    }
    */
}
