/*
  connectdialog.h - DM&P 86Scratch
  Copyright (c) 2018 Vic Chen <vic@dmp.com.tw>. All right reserved.
  Copyright (c) 2018 RoBoardGod <roboardgod@dmp.com.tw>. All right reserved.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License as
  published by the Free Software Foundation; either version 2 of
  the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston,
  MA  02110-1301  USA

  (If you need a commercial license, please contact soc@dmp.com.tw
   to get more information.)
*/

#ifndef CONNECTDIALOG_H
#define CONNECTDIALOG_H

#include "animefilehandle.h"
#include "tipspushbutton.h"
#include "tipsradiobutton.h"
#include "tipscombobox.h"
#include "tipscheckbox.h"

#include <QDialog>
#include <QCheckBox>
#include <QComboBox>
#include <QTimer>
#include <QRadioButton>
#include <QLabel>
#include <QLineEdit>

class MainWindow;


class ConnectDialog : public QDialog
{
    Q_OBJECT

public:
    ConnectDialog(MainWindow *in_parent = nullptr);

    int exec();
    void setName(QString);

private:
    //system
    QString name = "";
    MainWindow *parent = nullptr;
    bool direct = true;
    bool burning = false;
    void adjustFirmwareCb();
    void closeEvent(QCloseEvent *);


    //layout
    TipsCheckBox *burn_cb;
    TipsComboBox *firmware_cb = nullptr;
    QComboBox *com_cb = nullptr;
    QLabel *sepratate_bar = nullptr;
    TipsRadioButton *usb_serial_rb = nullptr;
    TipsComboBox *usb_serial_cb = nullptr;
    TipsRadioButton *esp8266_ap_rb = nullptr;
    QLabel *esp8266_ip = nullptr;
    QLabel *esp8266_password = nullptr;
    TipsRadioButton *nothing_rb = nullptr;

    TipsRadioButton *eth_auto_rb = nullptr;
    TipsComboBox *eth_auto_cb = nullptr;
    TipsRadioButton *eth_rb = nullptr;
    TipsComboBox *eth_cb = nullptr;

    QLineEdit *eth_ip_1 = nullptr;
    QLabel *point1 = nullptr;
    QLineEdit *eth_ip_2 = nullptr;
    QLabel *point2 = nullptr;
    QLineEdit *eth_ip_3 = nullptr;
    QLabel *point3 = nullptr;
    QLineEdit *eth_ip_4 = nullptr;

    QLabel *tips = nullptr;
    TipsPushButton *ok_pb = nullptr;
    TipsPushButton *cancel_pb = nullptr;

    //uploadin anime
    QTimer *uploading_anime_timer = nullptr;
    QLabel *uploading_anime = nullptr;
    QLabel *uploading_text = nullptr;
    AnimeFileHandle *uploading_anime_file = nullptr;

    //serial port detect
    QTimer *serial_detector = nullptr;

    //uploading
    QString firmware;
    QString uploadPort;
    QString previousPort;
    QTimer *wait_previous_port = nullptr;
    QTimer *scan = nullptr;
    void doUploadFirmware(QString firmware, QString port);
    void doScanFinish();


signals:
    void set_serial(QString);
    void set_lan(QString, QString);

private slots:
    void detect_serial_port();
    void close_dialog();
    void connect_pyhelper();

    void burn_cb_enter();
    void burn_cb_leave();

    void firmware_cb_enter();
    void firmware_cb_leave();
    void firmware_cb_click();
    void com_cb_enter();
    void com_cb_leave();
    void com_cb_click();
    void usb_serial_cb_enter();
    void usb_serial_cb_leave();
    void usb_serial_cb_click();
    void eth_auto_cb_enter();
    void eth_auto_cb_leave();
    void eth_auto_cb_click();
    void eth_cb_enter();
    void eth_cb_leave();
    void eth_cb_click();

    void usb_serial_rb_enter();
    void usb_serial_rb_leave();
    void esp8266_ap_rb_enter();
    void esp8266_ap_rb_leave();
    void eth_auto_rb_enter();
    void eth_auto_rb_leave();
    void eth_rb_enter();
    void eth_rb_leave();
    void nothing_rb_enter();
    void nothing_rb_leave();

    void ok_enter();
    void ok_leave();

    void cancel_enter();
    void cancel_leave();

    void do_uploading_anime();

    void do_reset_finish();
    void do_scan();
    void do_v86dude_finish();
    void do_check_previous_port();

    void ip1_text_changed(QString);
    void ip2_text_changed(QString);
    void ip3_text_changed(QString);
    void ip4_text_changed(QString);
};

#endif // CONNECTDIALOG_H
