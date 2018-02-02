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

#ifndef SERVER_H
#define SERVER_H

#include <QDialog>
#include <QTcpSocket>
#include <QDataStream>
#include <QProcess>

#include "mainwindow.h"

class QTcpServer;
class QTcpSocket;

class TCP86Client : public QDialog
{
    Q_OBJECT

public:

    explicit TCP86Client(MainWindow *p, int t_port = 8386,QWidget *parent = Q_NULLPTR);
    void sessionOpened(QStringList parameterList);

    QProcess *Python_process;
    QTimer *status_timer;

    int connect_status = -1;
    int connect_error = -1;
    int finish_status = 10;

    void set_Port(int t_port = 8386);
    void sendFortune(unsigned char);
    void TCPdisconnect();
    void status_Show(int status, QString *&str1, QString *&str2);
    void errorcode_Show(int errorcode, QString *&str1, QString *&str2);

signals:
    void helper_status(int status = 0x00);
    //86Scratch to Python helper ; Python helper to 86Duino board ; 86Duino board to Python helper ; Python helper to 86Scratch
    void helper_speed(int S2P = 0, int P2B = 0, int B2P = 0, int P2S = 0);
    void helper_IP(QString* IP_Addr);
    void helper_error(int errorcode = 0x00);

public slots:
    void readFortune();
    void displayError(QAbstractSocket::SocketError socketError);
    void status_update();


private:
    MainWindow *parent = nullptr;
    QTcpSocket *tcpSocket;
    int target_port = 8386;
    bool already = false;
};

#endif
