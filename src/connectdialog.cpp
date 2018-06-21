/* 
  connectdialog.cpp - DM&P 86Scratch
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

#include "connectdialog.h"
#include "mainwindow.h"

#include <QtSerialPort/QSerialPortInfo>
#include <QSerialPort>
#include <QDir>
#include <QApplication>
#include <QProcess>
#include <QCloseEvent>

#define cag(x) parent->checkAndGetValue(x)
#define cagInt(x) parent->checkAndGetValueInt(x)

ConnectDialog::ConnectDialog(MainWindow *in_parent)
{
    parent = in_parent;
    direct = true;
    this->setWindowIcon(QIcon(cag("WindowIconPath")));
}

/*
 * Overriding exec funtion
 */
int ConnectDialog::exec()
{
    int y = 0;
    QString str_style = cag("dialog_text_style_sheet");

    QLabel *tmp = new QLabel(this);
    tmp->setPixmap(QPixmap(cag("dialog_top_frame")));
    tmp->setGeometry(0, 0, cagInt("dialog_top_frame_width"), cagInt("dialog_top_frame_height"));
    tmp->setVisible(true);

    int times = 0;
    bool do_flag[6] = {0};
    if(name == "86Scratch")
    {
        times = 3;
    }
    else
    {
        times = 1;
        //1. look for firmware file
        QDir projectDir("./Projects/" + name);
        projectDir.setFilter(QDir::NoDotAndDotDot | QDir::Dirs);
        QStringList dirs = projectDir.entryList();

        QRegularExpression re_usb("USB$");
        QRegularExpression re_bt("bluetooth$");
        QRegularExpression re_eth("ethernet$");
        QRegularExpression re_wifi("wifi$");
        QRegularExpression re_esp("esp8266$");
        QRegularExpression re_espap("esp8266AP$");

        for(QString dir :dirs)
        {

            if(dir.contains(re_usb))
                do_flag[0] = true;
            else if(dir.contains(re_bt))
                do_flag[1] = true;
            else if(dir.contains(re_eth))
                do_flag[2] = true;
            else if(dir.contains(re_wifi))
                do_flag[3] = true;
            else if(dir.contains(re_esp))
                do_flag[4] = true;
            else if(dir.contains(re_espap))
                do_flag[5] = true;
            else
                continue;
        }

        if(do_flag[0] || do_flag[1])
            times++;
        if(do_flag[2] || do_flag[3] || do_flag[4])
            times+= 2;
        if(do_flag[5])
            times++;
    }

    for(int i = 0; i < times; ++i)
    {
        tmp = new QLabel(this);
        tmp->setPixmap(QPixmap(cag("dialog_center_frame")));
        tmp->setGeometry(0, cagInt("dialog_top_frame_height") + i * cagInt("dialog_center_frame_height"), cagInt("dialog_center_frame_width"), cagInt("dialog_center_frame_height"));
        tmp->setVisible(true);
    }

    tmp = new QLabel(this);
    tmp->setPixmap(QPixmap(cag("dialog_bottom_frame")));
    tmp->setGeometry(0, cagInt("dialog_top_frame_height") + times * cagInt("dialog_center_frame_height"), cagInt("dialog_bottom_frame_width"), cagInt("dialog_bottom_frame_height"));
    tmp->setVisible(true);

    // Allocate widgets
    sepratate_bar = new QLabel(this);
    sepratate_bar->setGeometry(cagInt("sepratate_bar_x_offset"), y + cagInt("sepratate_bar_y_offset"), cagInt("sepratate_bar_width"), cagInt("sepratate_bar_height"));
    sepratate_bar->setPixmap(QPixmap(cag("sepratate_bar_img")));
    sepratate_bar->setVisible(true);

    burn_cb = new TipsCheckBox(this);
    burn_cb->setText(cag("dailog_burn_cb_text"));
    burn_cb->setGeometry(cagInt("dailog_burn_cb_x"), cagInt("dailog_burn_cb_y"), cagInt("dailog_burn_cb_width"), cagInt("dailog_burn_cb_height"));
    burn_cb->setStyleSheet(str_style);
    burn_cb->setVisible(true);
    connect(burn_cb, SIGNAL(enter()), this, SLOT(burn_cb_enter()));
    connect(burn_cb, SIGNAL(leave()), this, SLOT(burn_cb_leave()));

    firmware_cb = new TipsComboBox(this);
    firmware_cb->setGeometry(cagInt("dailog_firmware_cb_x"), cagInt("dailog_firmware_cb_y"), cagInt("dailog_firmware_cb_width"), cagInt("dailog_firmware_cb_height"));
    firmware_cb->setStyleSheet(cag("dialog_cb_text_style_sheet"));
    firmware_cb->view()->setMinimumWidth(cagInt("dailog_firmware_cb_minimum_width"));
    firmware_cb->setVisible(true);
    connect(firmware_cb, SIGNAL(enter()), this, SLOT(firmware_cb_enter()));
    connect(firmware_cb, SIGNAL(leave()), this, SLOT(firmware_cb_leave()));
    connect(firmware_cb, SIGNAL(click()), this, SLOT(firmware_cb_click()));

    com_cb = new TipsComboBox(this);
    com_cb->setGeometry(cagInt("dailog_com_cb_x"), cagInt("dailog_com_cb_y"), cagInt("dailog_com_cb_width"), cagInt("dailog_com_cb_height"));
    com_cb->setStyleSheet(cag("dialog_cb_text_style_sheet"));
    com_cb->setVisible(true);
    connect(com_cb, SIGNAL(enter()), this, SLOT(com_cb_enter()));
    connect(com_cb, SIGNAL(leave()), this, SLOT(com_cb_leave()));
    connect(com_cb, SIGNAL(click()), this, SLOT(com_cb_click()));



    y+=cagInt("dialog_top_frame_height");

    serial_detector = new QTimer(this);
    connect(serial_detector, SIGNAL(timeout()), this, SLOT(detect_serial_port()));

    if(name == "86Scratch")
    {
        //1. look for firmware file
        QDir projectDir("./Projects/" + name);
        QStringList filenames = projectDir.entryList();
        for(QString filename : filenames)
        {
            if(filename.contains(".exe"))
            {
                firmware_cb->addItem(filename.left(filename.length() - 4));
            }
        }

        //2. adjust width
        adjustFirmwareCb();

        //3. layout
        usb_serial_rb = new TipsRadioButton(this);
        usb_serial_rb->setText(cag("dailog_serial_rb_text"));
        usb_serial_rb->setGeometry(cagInt("dailog_serial_rb_x"), y + cagInt("dailog_serial_rb_y"), cagInt("dailog_serial_rb_width"), cagInt("dailog_serial_rb_height"));
        usb_serial_rb->setStyleSheet(str_style);
        usb_serial_rb->setVisible(true);
        connect(usb_serial_rb, SIGNAL(enter()), this, SLOT(usb_serial_rb_enter()));
        connect(usb_serial_rb, SIGNAL(leave()), this, SLOT(usb_serial_rb_leave()));

        usb_serial_cb = new TipsComboBox(this);
        usb_serial_cb->setGeometry(cagInt("dailog_serial_cb_x"), y + cagInt("dailog_serial_cb_y"), cagInt("dailog_serial_cb_width"), cagInt("dailog_serial_cb_height"));
        usb_serial_cb->setStyleSheet(cag("dialog_cb_text_style_sheet"));
        usb_serial_cb->setVisible(true);
        connect(usb_serial_cb, SIGNAL(enter()), this, SLOT(usb_serial_cb_enter()));
        connect(usb_serial_cb, SIGNAL(leave()), this, SLOT(usb_serial_cb_leave()));
        connect(usb_serial_cb, SIGNAL(click()), this, SLOT(usb_serial_cb_click()));

        y+=cagInt("dialog_center_frame_height");

        //tmp = new QLabel(this);
        //tmp->setPixmap(QPixmap(cag("dialog_center_frame")));
        //tmp->setGeometry(0, y, cagInt("dialog_center_frame_width"), cagInt("dialog_center_frame_height"));
        //tmp->setVisible(true);

        esp8266_ap_rb = new TipsRadioButton(this);
        esp8266_ap_rb->setText(cag("dailog_esp8266ap_rb_text"));
        esp8266_ap_rb->setGeometry(cagInt("dailog_esp8266ap_rb_x"), y + cagInt("dailog_esp8266ap_rb_y"), cagInt("dailog_esp8266ap_rb_width"), cagInt("dailog_esp8266ap_rb_height"));
        esp8266_ap_rb->setStyleSheet(str_style);
        esp8266_ap_rb->setVisible(true);
        connect(esp8266_ap_rb, SIGNAL(enter()), this, SLOT(esp8266_ap_rb_enter()));
        connect(esp8266_ap_rb, SIGNAL(leave()), this, SLOT(esp8266_ap_rb_leave()));

        esp8266_ip = new QLabel(cag("dailog_esp8266ap_SSID"), this);
        esp8266_ip->setGeometry(cagInt("dailog_esp8266ap_SSID_x"), y + cagInt("dailog_esp8266ap_SSID_y"), cagInt("dailog_esp8266ap_SSID_width"), cagInt("dailog_esp8266ap_SSID_height"));
        esp8266_ip->setStyleSheet(str_style);
        esp8266_ip->setVisible(true);

        esp8266_password = new QLabel(cag("dailog_esp8266ap_password"), this);
        esp8266_password->setGeometry(cagInt("dailog_esp8266ap_password_x"), y + cagInt("dailog_esp8266ap_password_y"), cagInt("dailog_esp8266ap_password_width"), cagInt("dailog_esp8266ap_password_height"));
        esp8266_password->setStyleSheet(str_style);
        esp8266_password->setVisible(true);
        y += cagInt("dialog_center_frame_height");

        //tmp = new QLabel(this);
        //tmp->setPixmap(QPixmap(cag("dialog_center_frame")));
        //tmp->setGeometry(0, y, cagInt("dialog_center_frame_width"), cagInt("dialog_center_frame_height"));
        //tmp->setVisible(true);

        nothing_rb = new TipsRadioButton(this);
        nothing_rb->setText(cag("dailog_do_nothing_rb_text"));
        nothing_rb->setGeometry(cagInt("dailog_do_nothing_rb_x"), y + cagInt("dailog_do_nothing_rb_y"), cagInt("dailog_do_nothing_rb_width"), cagInt("dailog_do_nothing_rb_height"));
        nothing_rb->setChecked(true);
        nothing_rb->setStyleSheet(str_style);
        nothing_rb->setVisible(true);
        connect(nothing_rb, SIGNAL(enter()), this, SLOT(nothing_rb_enter()));
        connect(nothing_rb, SIGNAL(leave()), this, SLOT(nothing_rb_leave()));

        y += cagInt("dialog_center_frame_height");
    }
    else
    {
        //1. look for firmware file
        QDir projectDir("./Projects/" + name);
        projectDir.setFilter(QDir::NoDotAndDotDot | QDir::Dirs);
        QStringList dirs = projectDir.entryList();

        //QRegularExpression re_usb("USB$");
        //QRegularExpression re_bt("bluetooth$");
        //QRegularExpression re_eth("ethernet$");
        //QRegularExpression re_wifi("wifi$");
        //QRegularExpression re_esp("esp8266$");
        //QRegularExpression re_espap("esp8266AP$");


        for(QString dir :dirs)
        {

            //if(dir.contains(re_usb))
            //    do_flag[0] = true;
            //else if(dir.contains(re_bt))
            //    do_flag[1] = true;
            //else if(dir.contains(re_eth))
            //    do_flag[2] = true;
            //else if(dir.contains(re_wifi))
            //    do_flag[3] = true;
            //else if(dir.contains(re_esp))
            //    do_flag[4] = true;
            //else if(dir.contains(re_espap))
            //    do_flag[5] = true;
            //else
            //    continue;

            QDir firmDir("./Projects/" + name + "/" + dir);
            QStringList filter("*.exe");
            QStringList exes = firmDir.entryList(filter);

            for(QString exe : exes)
            {
                firmware_cb->addItem(exe.left(exe.length() - 4));
            }
        }

        //2.adjust width
        adjustFirmwareCb();

        //3. layout
        if(do_flag[0] || do_flag[1])
        {

            usb_serial_rb = new TipsRadioButton(this);
            usb_serial_rb->setText(cag("dailog_serial_rb_text"));
            usb_serial_rb->setGeometry(cagInt("dailog_serial_rb_x"), y + cagInt("dailog_serial_rb_y"), cagInt("dailog_serial_rb_width"), cagInt("dailog_serial_rb_height"));
            usb_serial_rb->setStyleSheet(str_style);
            usb_serial_rb->setVisible(true);
            connect(usb_serial_rb, SIGNAL(enter()), this, SLOT(usb_serial_rb_enter()));
            connect(usb_serial_rb, SIGNAL(leave()), this, SLOT(usb_serial_rb_leave()));

            usb_serial_cb = new TipsComboBox(this);
            usb_serial_cb->setGeometry(cagInt("dailog_serial_cb_x"), y + cagInt("dailog_serial_cb_y"), cagInt("dailog_serial_cb_width"), cagInt("dailog_serial_cb_height"));
            usb_serial_cb->setStyleSheet(cag("dialog_cb_text_style_sheet"));
            usb_serial_cb->setVisible(true);
            connect(usb_serial_cb, SIGNAL(enter()), this, SLOT(usb_serial_cb_enter()));
            connect(usb_serial_cb, SIGNAL(leave()), this, SLOT(usb_serial_cb_leave()));
            connect(usb_serial_cb, SIGNAL(click()), this, SLOT(usb_serial_cb_click()));

            y+=cagInt("dialog_center_frame_height");
        }
        for(int i = 2; i < 5; ++i)
        {
            if(do_flag[i] == false)
                continue;
            if(eth_rb == nullptr)
            {
                //tmp = new QLabel(this);
                //tmp->setPixmap(QPixmap(cag("dialog_center_frame")));
                //tmp->setGeometry(0, y, cagInt("dialog_center_frame_width"), cagInt("dialog_center_frame_height"));
                //tmp->setVisible(true);

                eth_auto_rb = new TipsRadioButton(this);
                eth_auto_rb->setText(cag("dailog_eth_auto_rb_text"));
                eth_auto_rb->setGeometry(cagInt("dailog_eth_auto_rb_x"), y + cagInt("dailog_eth_auto_rb_y"), cagInt("dailog_eth_auto_rb_width"), cagInt("dailog_eth_auto_rb_height"));
                eth_auto_rb->setStyleSheet(str_style);
                eth_auto_rb->setVisible(true);
                connect(eth_auto_rb, SIGNAL(enter()), this, SLOT(eth_auto_rb_enter()));
                connect(eth_auto_rb, SIGNAL(leave()), this, SLOT(eth_auto_rb_leave()));


                eth_auto_cb = new TipsComboBox(this);
                eth_auto_cb->setGeometry(cagInt("dailog_eth_auto_cb_x"), y + cagInt("dailog_eth_auto_cb_y"), cagInt("dailog_eth_auto_cb_width"), cagInt("dailog_eth_auto_cb_height"));
                eth_auto_cb->setStyleSheet(cag("dialog_cb_text_style_sheet"));
                eth_auto_cb->setVisible(true);
                connect(eth_auto_cb, SIGNAL(enter()), this, SLOT(eth_auto_cb_enter()));
                connect(eth_auto_cb, SIGNAL(leave()), this, SLOT(eth_auto_cb_leave()));
                connect(eth_auto_cb, SIGNAL(click()), this, SLOT(eth_auto_cb_click()));

                y += cagInt("dialog_center_frame_height");

                //tmp = new QLabel(this);
                //tmp->setPixmap(QPixmap(cag("dialog_center_frame")));
                //tmp->setGeometry(0, y, cagInt("dialog_center_frame_width"), cagInt("dialog_center_frame_height"));
                //tmp->setVisible(true);

                eth_rb = new TipsRadioButton(this);
                eth_rb->setText(cag("dailog_eth_rb_text"));
                eth_rb->setGeometry(cagInt("dailog_eth_rb_x"), y + cagInt("dailog_eth_rb_y"), cagInt("dailog_eth_rb_width"), cagInt("dailog_eth_rb_height"));
                eth_rb->setStyleSheet(str_style);
                eth_rb->setVisible(true);
                connect(eth_rb, SIGNAL(enter()), this, SLOT(eth_rb_enter()));
                connect(eth_rb, SIGNAL(leave()), this, SLOT(eth_rb_leave()));

                eth_cb = new TipsComboBox(this);
                eth_cb->setGeometry(cagInt("dailog_eth_cb_x"), y + cagInt("dailog_eth_cb_y"), cagInt("dailog_eth_cb_width"), cagInt("dailog_eth_cb_height"));
                eth_cb->setStyleSheet(cag("dialog_cb_text_style_sheet"));
                eth_cb->setVisible(true);
                connect(eth_cb, SIGNAL(enter()), this, SLOT(eth_cb_enter()));
                connect(eth_cb, SIGNAL(leave()), this, SLOT(eth_cb_leave()));
                connect(eth_cb, SIGNAL(click()), this, SLOT(eth_cb_click()));

                int x = cagInt("dailog_ip_x");
                eth_ip_1 = new QLineEdit(this);
                eth_ip_1->setGeometry(cagInt("dailog_ip_x"), y + cagInt("dailog_ip_y"), cagInt("dailog_ip_width"), cagInt("dailog_ip_height"));
                eth_ip_1->setStyleSheet(cag("dailog_eth_ip_text_style_sheet"));
                eth_ip_1->setMaxLength(3);
                eth_ip_1->setVisible(true);
                eth_ip_1->setValidator( new QIntValidator(0, 255, this) );
                connect(eth_ip_1, SIGNAL(textChanged(QString)), this, SLOT(ip1_text_changed(QString)));

                x += cagInt("dailog_ip_x_step");
                point1 = new QLabel(".", this);
                point1->setGeometry(x, y + cagInt("dailog_point_y"), cagInt("dailog_point_width"), cagInt("dailog_point_height"));
                point1->setStyleSheet(str_style);
                point1->setVisible(true);

                x += cagInt("dailog_point_x_step");
                eth_ip_2 = new QLineEdit(this);
                eth_ip_2->setGeometry(x, y + cagInt("dailog_ip_y"), cagInt("dailog_ip_width"), cagInt("dailog_ip_height"));
                eth_ip_2->setStyleSheet(cag("dailog_eth_ip_text_style_sheet"));
                eth_ip_2->setMaxLength(3);
                eth_ip_2->setVisible(true);
                eth_ip_2->setValidator( new QIntValidator(0, 255, this) );
                connect(eth_ip_2, SIGNAL(textChanged(QString)), this, SLOT(ip2_text_changed(QString)));

                x += cagInt("dailog_ip_x_step");
                point2 = new QLabel(".", this);
                point2->setGeometry(x, y + cagInt("dailog_point_y"), cagInt("dailog_point_width"), cagInt("dailog_point_height"));
                point2->setStyleSheet(str_style);
                point2->setVisible(true);

                x += cagInt("dailog_point_x_step");
                eth_ip_3 = new QLineEdit(this);
                eth_ip_3->setGeometry(x, y + cagInt("dailog_ip_y"), cagInt("dailog_ip_width"), cagInt("dailog_ip_height"));
                eth_ip_3->setStyleSheet(cag("dailog_eth_ip_text_style_sheet"));
                eth_ip_3->setMaxLength(3);
                eth_ip_3->setVisible(true);
                eth_ip_3->setValidator( new QIntValidator(0, 255, this) );
                connect(eth_ip_3, SIGNAL(textChanged(QString)), this, SLOT(ip3_text_changed(QString)));

                x += cagInt("dailog_ip_x_step");
                point3 = new QLabel(".", this);
                point3->setGeometry(x, y + cagInt("dailog_point_y"), cagInt("dailog_point_width"), cagInt("dailog_point_height"));
                point3->setStyleSheet(str_style);
                point3->setVisible(true);

                x += cagInt("dailog_point_x_step");
                eth_ip_4 = new QLineEdit(this);
                eth_ip_4->setGeometry(x, y + cagInt("dailog_ip_y"), cagInt("dailog_ip_width"), cagInt("dailog_ip_height"));
                eth_ip_4->setStyleSheet(cag("dailog_eth_ip_text_style_sheet"));
                eth_ip_4->setMaxLength(3);
                eth_ip_4->setVisible(true);
                eth_ip_4->setValidator( new QIntValidator(0, 255, this) );
                connect(eth_ip_4, SIGNAL(textChanged(QString)), this, SLOT(ip4_text_changed(QString)));

                y += cagInt("dialog_center_frame_height");
            }

            if(i == 2)
            {
                eth_auto_cb->addItem("Ethernet");
                eth_cb->addItem("Ethernet");
            }
            else if(i == 3)
            {
                eth_auto_cb->addItem("Wifi");
                eth_cb->addItem("Wifi");
            }
            else if(i == 4)
            {
                eth_auto_cb->addItem("ESP8266");
                eth_cb->addItem("ESP8266");
            }
        }

        if(do_flag[5])
        {
            //tmp = new QLabel(this);
            //tmp->setPixmap(QPixmap(cag("dialog_center_frame")));
            //tmp->setGeometry(0, y, cagInt("dialog_center_frame_width"), cagInt("dialog_center_frame_height"));
            //tmp->setVisible(true);

            esp8266_ap_rb = new TipsRadioButton(this);
            esp8266_ap_rb->setText(cag("dailog_esp8266ap_rb_text"));
            esp8266_ap_rb->setGeometry(cagInt("dailog_esp8266ap_rb_x"), y + cagInt("dailog_esp8266ap_rb_y"), cagInt("dailog_esp8266ap_rb_width"), cagInt("dailog_esp8266ap_rb_height"));
            esp8266_ap_rb->setStyleSheet(str_style);
            esp8266_ap_rb->setVisible(true);
            connect(esp8266_ap_rb, SIGNAL(enter()), this, SLOT(esp8266_ap_rb_enter()));
            connect(esp8266_ap_rb, SIGNAL(leave()), this, SLOT(esp8266_ap_rb_leave()));

            y += cagInt("dialog_center_frame_height");
        }

        //tmp = new QLabel(this);
        //tmp->setPixmap(QPixmap(cag("dialog_center_frame")));
        //tmp->setGeometry(0, y, cagInt("dialog_center_frame_width"), cagInt("dialog_center_frame_height"));
        //tmp->setVisible(true);

        nothing_rb = new TipsRadioButton(this);
        nothing_rb->setText(cag("dailog_do_nothing_rb_text"));
        nothing_rb->setGeometry(cagInt("dailog_do_nothing_rb_x"), y + cagInt("dailog_do_nothing_rb_y"), cagInt("dailog_do_nothing_rb_width"), cagInt("dailog_do_nothing_rb_height"));
        nothing_rb->setChecked(true);
        nothing_rb->setStyleSheet(str_style);
        nothing_rb->setVisible(true);
        connect(nothing_rb, SIGNAL(enter()), this, SLOT(nothing_rb_enter()));
        connect(nothing_rb, SIGNAL(leave()), this, SLOT(nothing_rb_leave()));

        y += cagInt("dialog_center_frame_height");

    }

    ok_pb = new TipsPushButton(this);
    ok_pb->setText(cag("dailog_ok_text"));

    cancel_pb = new TipsPushButton(this);
    cancel_pb->setText(cag("dialog_cancel_text"));

    tips = new QLabel(cag("dailog_tips_text"), this);

    ok_pb->setGeometry(cagInt("dailog_ok_x_offset"), y + cagInt("dailog_ok_y_offset"), cagInt("dailog_ok_width"), cagInt("dailog_ok_height"));
    ok_pb->setStyleSheet(cag("dailog_button_style_sheet"));
    ok_pb->setVisible(true);

    cancel_pb->setGeometry(cagInt("dailog_cancel_x_offset"), y + cagInt("dailog_cancel_y_offset"), cagInt("dailog_cancel_width"), cagInt("dailog_cancel_height"));
    cancel_pb->setStyleSheet(cag("dailog_button_style_sheet"));
    cancel_pb->setVisible(true);

    tips->setWordWrap(true);
    tips->setGeometry(cagInt("dailog_tips_x_offset"), y + cagInt("dailog_tips_y_offset"), cagInt("dailog_tips_width"), cagInt("dailog_tips_height"));
    tips->setStyleSheet(cag("dailog_tips_style_sheet"));
    tips->setVisible(true);

    QPixmap bkgnd(cag("ConnectDialogGB"));
    QRect rect(200, (450 - y) / 2, 520, (450 + y) / 2);
    bkgnd = bkgnd.copy(rect);
    QPalette palette;
    palette.setBrush(QPalette::Background, bkgnd);
    this->setPalette(palette);

    connect(cancel_pb, SIGNAL(clicked()), this, SLOT(close_dialog()));
    connect(cancel_pb, SIGNAL(enter()), this, SLOT(cancel_enter()));
    connect(cancel_pb, SIGNAL(leave()), this, SLOT(cancel_leave()));

    connect(ok_pb, SIGNAL(clicked()), this, SLOT(connect_pyhelper()));
    connect(ok_pb, SIGNAL(enter()), this, SLOT(ok_enter()));
    connect(ok_pb, SIGNAL(leave()), this, SLOT(ok_leave()));

    this->setFixedSize(430, y + 90);

    serial_detector->start(cagInt("dialog_scan_com_port_ms"));
    return QDialog::exec();
}

/*
 * Set Project Name
 */
void ConnectDialog::setName(QString in)
{
    name = in;
}

/*
 * Upload firmware
 */
void ConnectDialog::doUploadFirmware(QString firmware, QString port)
{
    QProcess *reseter = new QProcess(this);
#if defined(Q_OS_LINUX) && defined(Q_PROCESSOR_X86_32)
    QFile reset86("./uploader/reset_linux32");
    QFile v86dude("./uploader/v86dude_linux32");
    if (v86dude.exists() && reset86.exists())
    {
        port = port.toLower();
        reseter->start(reset86.fileName(), QStringList() << "/dev/" + port);
        connect(reseter, SIGNAL(finished(int,QProcess::ExitStatus)), this, SLOT(do_reset_finish()));
    }
    else
    {
        parent->warnMessage(cag("noUploadder"));
        parent->dialog_exception = true;
        uploading_anime_timer->stop();
        burning = false;
    }

#elif defined(Q_OS_LINUX) && defined(Q_PROCESSOR_X86_64)
    QFile reset86("./uploader/reset_linux64");
    QFile v86dude("./uploader/v86dude_linux64");
    if (v86dude.exists() && reset86.exists())
    {
        port = port.toLower();
        reseter->start(reset86.fileName(), QStringList() << "/dev/" + port);
        connect(reseter, SIGNAL(finished(int,QProcess::ExitStatus)), this, SLOT(do_reset_finish()));
    }
    else
    {
        parent->warnMessage(cag("noUploadder"));
        parent->dialog_exception = true;
        uploading_anime_timer->stop();
        burning = false;
    }
#elif defined(Q_OS_MACOS)
    QFile reset86("./uploader/reset_macosx.exe");
    QFile v86dude("./uploader/v86dude_mac.exe");
    if (v86dude.exists() && reset86.exists())
    {
        port = port.toLower();
        reseter->start(reset86.fileName(), QStringList() << "/dev/" + port);
        connect(reseter, SIGNAL(finished(int,QProcess::ExitStatus)), this, SLOT(do_reset_finish()));
    }
    else
    {
        parent->warnMessage(cag("noUploadder"));
        parent->dialog_exception = true;
        uploading_anime_timer->stop();
        burning = false;
    }
#elif defined(Q_OS_WIN)
    QFile reset86("./uploader/reset_win.exe");
    QFile v86dude("./uploader/v86dude_win.exe");
    if (v86dude.exists() && reset86.exists())
    {
        port = port.toLower();
        reseter->start(reset86.fileName(), QStringList() << port);
        connect(reseter, SIGNAL(finished(int,QProcess::ExitStatus)), this, SLOT(do_reset_finish()));
    }
    else
    {
        parent->warnMessage(cag("noUploadder"));
        parent->dialog_exception = true;
        uploading_anime_timer->stop();
        burning = false;
    }
#endif
}

/*
 * Scan new port finished
 */
void ConnectDialog::doScanFinish()
{
    scan->stop();


    QProcess *p_86dude = new QProcess(this);
    if (uploadPort != "")
    {
        uploadPort = uploadPort.toLower();
        p_86dude->setWorkingDirectory(QDir::currentPath());
#if defined(Q_OS_LINUX) && defined(Q_PROCESSOR_X86_32)
        QFile v86dude("./uploader/v86dude_linux32");
        p_86dude->start(v86dude.fileName(), QStringList() << "/dev/" + uploadPort << "20" << firmware);
#elif defined(Q_OS_LINUX) && defined(Q_PROCESSOR_X86_64)
        QFile v86dude("./uploader/v86dude_linux64");
        p_86dude->start(v86dude.fileName(), QStringList() << "/dev/" + uploadPort << "20" << firmware);
#elif defined(Q_OS_MACOS)
        QFile v86dude("./uploader/v86dude_mac.exe");
        p_86dude->start(v86dude.fileName(), QStringList() << "/dev/" + uploadPort << "20" << firmware);
#elif defined(Q_OS_WIN)
        QFile v86dude("./uploader/v86dude_win.exe");
        p_86dude->start(v86dude.fileName(), QStringList() << uploadPort << "20" << firmware);
#endif
        connect(p_86dude, SIGNAL(finished(int,QProcess::ExitStatus)), this, SLOT(do_v86dude_finish()));

    }
    else
    {
        parent->warnMessage(cag("noUploadPort"));
        uploading_anime_timer->stop();
        uploading_anime->setVisible(false);
        uploading_text->setVisible(false);
        delete uploading_anime;
        delete uploading_text;
        serial_detector->start(cagInt("dialog_scan_com_port_ms"));
        ok_pb->setEnabled(true);
        cancel_pb->setEnabled(true);
        burning = false;
    }

}

/*
 * Adjust firmware combo box via content
 */
void ConnectDialog::adjustFirmwareCb()
{
    int scroll = firmware_cb->count() <= firmware_cb->maxVisibleItems() ? 0 :
    QApplication::style()->pixelMetric(QStyle::PixelMetric::PM_ScrollBarExtent);

    int max = 0;

    for (int i = 0; i < firmware_cb->count(); i++)
    {
        int width = firmware_cb->view()->fontMetrics().width(firmware_cb->itemText(i));
        if (max < width)max = width;
    }

    firmware_cb->view()->setMinimumWidth(scroll + max);
}

/*
 * Overriding close event
 */
void ConnectDialog::closeEvent(QCloseEvent *event)
{
    if(burning)
        event->ignore();
    if(serial_detector->isActive())
        serial_detector->stop();
    if(direct)
        parent->dialog_exception = true;
}

// utils
/*
 * Remove Invalid Com ports
 */
inline void removeInvalidPorts(QList<QSerialPortInfo>& infos)
{
#if defined(Q_OS_MACOS)
    for (int i = 0; i < infos.count(); i++)
    {
        if (infos.at(i).portName().contains("tty"))
            infos.removeAt(i);
    }
#endif
}


//slots

/*
 * Detect Avalible Com port
 */
void ConnectDialog::detect_serial_port()
{
    QString currentPort = com_cb->currentText();
    QString currentPortUSB;

    if(usb_serial_cb != nullptr)
    {
        currentPortUSB = usb_serial_cb->currentText();
    }


    auto infos = QSerialPortInfo::availablePorts();
    removeInvalidPorts(infos);

    for(int i = 0; i < com_cb->count(); i++)
    {
        bool isContain = false;
        for(const QSerialPortInfo &info : infos)
        {
            if(info.portName() == com_cb->itemText(i))
            {
                isContain = true;
                break;
            }
        }
        if(isContain == false)
        {
            com_cb->removeItem(i);
            if(usb_serial_cb != nullptr)
            {
                usb_serial_cb->removeItem(i);
            }
        }
    }

    for(const QSerialPortInfo &info : infos)
    {
        if(com_cb->findText(info.portName()) == -1)
        {
            com_cb->addItem(info.portName());
            if(usb_serial_cb != nullptr)
            {
                usb_serial_cb->addItem(info.portName());
            }
        }
    }

    auto findIndex_com = com_cb->findText(currentPort);
    int findIndex_usb;
    if(usb_serial_cb != nullptr)
     findIndex_usb = usb_serial_cb->findText(currentPortUSB);

    if(com_cb->count() == 1)
    {
        com_cb->setCurrentIndex(0);
        if(usb_serial_cb != nullptr)
        {
            usb_serial_cb->setCurrentIndex(0);
        }
    }
    else
    {
        com_cb->setCurrentIndex(findIndex_com);
        if(usb_serial_cb != nullptr)
        {
            usb_serial_cb->setCurrentIndex(findIndex_usb);
        }
    }
    if(com_cb->currentIndex() == -1 && com_cb->count() != 0)
    {
        com_cb->setCurrentIndex(0);
    }
    if(usb_serial_cb != nullptr && usb_serial_cb->currentIndex() == -1 && usb_serial_cb->count() != 0)
    {
        usb_serial_cb->setCurrentIndex(0);
    }
}

/*
 * Cancel pressed event
 */
void ConnectDialog::close_dialog()
{
    direct = true;
    this->close();
}

/*
 * OK pressed event
 */
void ConnectDialog::connect_pyhelper()
{
    serial_detector->stop();
    direct = false;

    //1. set parameter
    if(usb_serial_rb != nullptr && usb_serial_rb->isChecked())
    {
        QString port = usb_serial_cb->currentText();
        emit set_serial(port);
        if(parent->dialog_exception)
        {
            serial_detector->start(cagInt("dialog_scan_com_port_ms"));
            return;
        }
    }
    else if(eth_auto_rb != nullptr && eth_auto_rb->isChecked())
    {
        emit set_lan("autodetect", "2000");
        if(parent->dialog_exception)
        {
            serial_detector->start(cagInt("dialog_scan_com_port_ms"));
            return;
        }
    }
    else if(esp8266_ap_rb != nullptr && esp8266_ap_rb->isChecked())
    {
        emit set_lan("192.168.4.1", "2000");
        if(parent->dialog_exception)
        {
            serial_detector->start(cagInt("dialog_scan_com_port_ms"));
            return;
        }
    }
    else if(eth_rb != nullptr && eth_rb->isChecked())
    {
        QString ip = eth_ip_1->text() + "." + eth_ip_2->text() + "." + eth_ip_3->text() + "." + eth_ip_4->text();
        emit set_lan(ip, "2000");
        if(parent->dialog_exception)
        {
            serial_detector->start(cagInt("dialog_scan_com_port_ms"));
            return;
        }
    }
    if(parent->dialog_exception)
    {
        serial_detector->start(cagInt("dialog_scan_com_port_ms"));
        parent->dialog_exception = false;
        return;
    }


    //2. Start Burning Thread
    if(burn_cb->isChecked())
    {
        if(com_cb->currentText() == "")
        {
            parent->warnMessage("Error: Missing upload port");
            parent->dialog_exception = true;
            return;
        }

        burning = true;
        QSize s = this->size();
        uploading_anime_file = new AnimeFileHandle(cag("UploadingAnime"), true);
        uploading_anime_file->resetIndex();
        uploading_anime = new QLabel(this);
        uploading_anime->setGeometry(s.width()/2 - 150, s.height()/2 - 100, 300, 200);
        uploading_anime->setVisible(true);
        uploading_anime_timer = new QTimer(this);
        connect(uploading_anime_timer, SIGNAL(timeout()), this, SLOT(do_uploading_anime()));
        uploading_anime_timer->start(cagInt("uploading_anime_delay"));

        uploading_text = new QLabel(cag("uploading_text"), this);
        uploading_text->setGeometry(s.width()/2-150+cagInt("uploading_text_x_offset"), s.height()/2-100+cagInt("uploading_text_y_offset"), cagInt("uploading_text_width"), cagInt("uploading_text_height"));
        uploading_text->setStyleSheet(cag("uploading_text_style_sheet"));
        uploading_text->setVisible(true);

        ok_pb->setEnabled(false);
        cancel_pb->setEnabled(false);

        if(name == "86Scratch")
        {
            //upload firmware
            firmware = "./Projects/" + name + "/" + firmware_cb->currentText() + ".exe";
            QString port = com_cb->currentText();
            previousPort = port;
            doUploadFirmware(firmware, port);
        }
        else
        {
            QDir projectDir("./Projects/" + name);
            projectDir.setFilter(QDir::NoDotAndDotDot | QDir::Dirs);
            QStringList dirs = projectDir.entryList();

            for(QString dir : dirs)
            {
                QDir firmDir("./Projects/" + name + "/" + dir);
                QStringList filter(firmware_cb->currentText() + ".exe");
                QStringList exes = firmDir.entryList(filter);

                if(exes.count() != 0)
                {
                    QString port = com_cb->currentText();
                    previousPort = port;
                    firmware = "./Projects/" + name + "/" + dir + "/" + firmware_cb->currentText() + ".exe";
                    doUploadFirmware(firmware, port);
                    break;
                }
            }
        }
    }


    if(nothing_rb->isChecked())
    {
        direct = true;
        close_dialog();
    }
    this->close();

}

/*
 * Upload anime timer handler
 */
void ConnectDialog::do_uploading_anime()
{
    QPixmap img(uploading_anime_file->getNextImagePath());
    uploading_anime->setPixmap(img);
}

/*
 * Reset port finished handler
 */
void ConnectDialog::do_reset_finish()
{
    scan = new QTimer(this);
    connect(scan, SIGNAL(timeout()), this, SLOT(do_scan()));
    scan->start(cagInt("dialog_scan_new_port_delay_ms"));
}

/*
 * Scan new port for uploading.
 */
void ConnectDialog::do_scan()
{
    static int i = 1;
    static int total = cagInt("dialog_scan_new_port_timeout_ms")/cagInt("dialog_scan_new_port_delay_ms");
    static QStringList currentPorts;
    static auto infos = QSerialPortInfo::availablePorts();

    if(i == 1)
    {
        infos = QSerialPortInfo::availablePorts();
        removeInvalidPorts(infos);

        for (const QSerialPortInfo &info : infos)
            currentPorts.push_back(info.portName());
    }


    infos = QSerialPortInfo::availablePorts();
    removeInvalidPorts(infos);
    for (const QSerialPortInfo &info : infos)
    {
        if (!currentPorts.contains(info.portName()))
        {
            uploadPort = info.portName();
            i = 1;
            scan->stop();
            currentPorts.clear();
            doScanFinish();
        }
    }

    i++;
    if(i == total)
    {
        //time out
        i = 1;
        scan->stop();
        uploadPort = "";
        currentPorts.clear();
        doScanFinish();
    }
}

/*
 * uploading finished handle, start scan previous port
 */
void ConnectDialog::do_v86dude_finish()
{
    // Wait com port back to normal.
    wait_previous_port = new QTimer(this);
    connect(wait_previous_port, SIGNAL(timeout()), this, SLOT(do_check_previous_port()));
    wait_previous_port->start(cagInt("dialog_scan_previous_port_delay_ms"));
}

/*
 * scan previous port
 */
void ConnectDialog::do_check_previous_port()
{
    static int i = 0;
    static int total = cagInt("dialog_scan_previous_port_timeout_ms") / cagInt("dialog_scan_previous_port_delay_ms");
    wait_previous_port->stop();

    auto infos = QSerialPortInfo::availablePorts();
    removeInvalidPorts(infos);
    for (const QSerialPortInfo &info : infos)
    {
        if (info.portName() == previousPort)
        {
            if(burning || i == total)
            {
                i = 0;
                burning = false;
                wait_previous_port->stop();
                if(i != total)
                {
                    parent->warnMessage(cag("uploadFinish"));
                    parent->dialog_exception = false;
                }
                else
                {
                    parent->warnMessage(cag("uploadFinishNoPreviousPort"));
                    parent->dialog_exception = true;
                }
                uploading_anime_timer->stop();
                uploading_anime->setVisible(false);
                uploading_text->setVisible(false);
                ok_pb->setEnabled(true);
                cancel_pb->setEnabled(true);
                delete uploading_text;
                delete uploading_anime;

                this->close();
            }
        }
    }
    i++;
    wait_previous_port->start(cagInt("dialog_scan_previous_port_delay_ms"));
}

/*
 * OK button enter event
 */
void ConnectDialog::ok_enter()
{
    tips->setText(cag("dailog_ok_enter_tip"));
}

/*
 * OK button leave event
 */
void ConnectDialog::ok_leave()
{
    tips->setText(cag("dailog_tips_text"));
}

/*
 * Cancel button enter event
 */
void ConnectDialog::cancel_enter()
{
    tips->setText(cag("dailog_cancel_enter_tip"));
}

/*
 * Cancel button leave event
 */
void ConnectDialog::cancel_leave()
{
    tips->setText(cag("dailog_tips_text"));
}

/*
 * USB Radio button enter event
 */
void ConnectDialog::usb_serial_rb_enter()
{
    tips->setText(cag("dailog_serial_rb_enter_tip"));
}

/*
 * USB Radio button leave event
 */
void ConnectDialog::usb_serial_rb_leave()
{
    tips->setText(cag("dailog_tips_text"));
}

/*
 * ESP8266 AP mode Radio button enter event
 */
void ConnectDialog::esp8266_ap_rb_enter()
{
    tips->setText(cag("dailog_esp8266ap_rb_enter_tip"));
}

/*
 * ESP8266 AP mode Radio button leave event
 */
void ConnectDialog::esp8266_ap_rb_leave()
{
    tips->setText(cag("dailog_tips_text"));
}

/*
 * Nothing Radio button enter event
 */
void ConnectDialog::nothing_rb_enter()
{
    tips->setText(cag("dailog_do_nothing_rb_enter_tip"));
}

/*
 * Nothing Radio button leave event
 */
void ConnectDialog::nothing_rb_leave()
{
    tips->setText(cag("dailog_tips_text"));
}

/*
 * Ethernet Auto radiobutton enter event
 */
void ConnectDialog::eth_auto_rb_enter()
{
    tips->setText(cag("dailog_eth_auto_rb_enter"));
}

/*
 * Ethernet Auto radiobutton leave event
 */
void ConnectDialog::eth_auto_rb_leave()
{
    tips->setText(cag("dailog_tips_text"));
}

/*
 * Ethernet radiobutton enter event
 */
void ConnectDialog::eth_rb_enter()
{
    tips->setText(cag("dailog_eth_rb_enter"));
}

/*
 * Ethernet radiobutton leave event
 */
void ConnectDialog::eth_rb_leave()
{
    tips->setText(cag("dailog_tips_text"));
}

/*
 * firmware combobox enter event
 */
void ConnectDialog::firmware_cb_enter()
{
    tips->setText(cag("dailog_firmware_cb_enter_tips"));
}

/*
 * firmware combobox leave event
 */
void ConnectDialog::firmware_cb_leave()
{
    tips->setText(cag("dailog_tips_text"));
}

/*
 * firmware combobox click event
 */
void ConnectDialog::firmware_cb_click()
{
    burn_cb->setChecked(true);
}

/*
 * com combobox enter event
 */
void ConnectDialog::com_cb_enter()
{
    tips->setText(cag("dailog_com_cb_enter_tips"));
}

/*
 * com combobox leave event
 */
void ConnectDialog::com_cb_leave()
{
    tips->setText(cag("dailog_tips_text"));
}

/*
 * com combobox click event
 */
void ConnectDialog::com_cb_click()
{
    burn_cb->setChecked(true);
}


/*
 * Serial combobox enter event
 */
void ConnectDialog::usb_serial_cb_enter()
{
    tips->setText(cag("dailog_serial_cb_enter_tips"));
}

/*
 * Serial combobox leave event
 */
void ConnectDialog::usb_serial_cb_leave()
{
    tips->setText(cag("dailog_tips_text"));
}

/*
 * Serial combobox click event
 */
void ConnectDialog::usb_serial_cb_click()
{
    usb_serial_rb->setChecked(true);
}


/*
 * Ethernet autodetect combobox enter event
 */
void ConnectDialog::eth_auto_cb_enter()
{
    tips->setText(cag("dailog_eth_auto_cb_enter_tips"));
}

/*
 * Ethernet autodetect combobox leave event
 */
void ConnectDialog::eth_auto_cb_leave()
{
    tips->setText(cag("dailog_tips_text"));
}

/*
 * Ethernet autodetect combobox click event
 */
void ConnectDialog::eth_auto_cb_click()
{
    eth_auto_rb->setChecked(true);
}


/*
 * Ethernet combobox enter event
 */
void ConnectDialog::eth_cb_enter()
{
    tips->setText(cag("dailog_eth_cb_enter_tips"));
}

/*
 * Ethernet combobox leave event
 */
void ConnectDialog::eth_cb_leave()
{
    tips->setText(cag("dailog_tips_text"));
}

/*
 * Ethernet  combobox click event
 */
void ConnectDialog::eth_cb_click()
{
    eth_rb->setChecked(true);
}


/*
 * burn checkbox enter event
 */
void ConnectDialog::burn_cb_enter()
{
    tips->setText(cag("dailog_burn_cb_enter_tips"));
}

/*
 * burn checkbox leave event
 */
void ConnectDialog::burn_cb_leave()
{
    tips->setText(cag("dailog_tips_text"));
}

/*
 * IP Line Edit 1 text changed
 */
void ConnectDialog::ip1_text_changed(QString text)
{
    eth_rb->setChecked(true);
    if(text.length() == 3)
    {
        eth_ip_2->setFocus();
    }
}

/*
 * IP Line Edit 2 text changed
 */
void ConnectDialog::ip2_text_changed(QString text)
{
    eth_rb->setChecked(true);
    if(text.length() == 3)
    {
        eth_ip_3->setFocus();
    }
}

/*
 * IP Line Edit 3 text changed
 */
void ConnectDialog::ip3_text_changed(QString text)
{
    eth_rb->setChecked(true);
    if(text.length() == 3)
    {
        eth_ip_4->setFocus();
    }
}


/*
 * IP Line Edit 4 text changed
 */
void ConnectDialog::ip4_text_changed(QString text)
{
    eth_rb->setChecked(true);
}
