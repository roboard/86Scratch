/* 
  mainwindow.h - DM&P 86Scratch
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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "inihandler.h"
#include "tipspushbutton.h"
#include "animefilehandle.h"
#include "connectdialog.h"


#include <QMainWindow>
#include <QListWidget>
#include <QProcess>
#include <QTextEdit>
#include <QLabel>
#include <QTimer>
#include <QDir>

class TCP86Client;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

    bool show();
    QString checkAndGetValue(QString key);
    int checkAndGetValueInt(QString key);
    void warnMessage(QString text);

    bool dialog_exception = false;

private:
    //system objects
    QFont *o_font;
    QProcess *scratch;
    bool init_fail = true;
    bool running = false;
    INIHandler *o_ini = nullptr;
    ConnectDialog *dialog = nullptr;
    QStringList parameterList;
    TCP86Client *client = nullptr;


    //methods
    void drawBG(QString file);
    bool helperCopy();
    void iniErrorMessage(QString key);
    void iniIntErrorMessage(QString key);
    void setFont(QString file);
    QString defaultProjectName(QString defaultBase = "NewProject");
    bool containedProject(QString name);
    bool copyDirRecursively(QString sourceFolder, QString destFolder);
    void updateProjectList();
    void loadProjectData(int row);
    void launchScratch(const QDir& sb2, const QStringList& sb2Files);
    int measureTextWidth(QString str, QString font_path, int font_size);

    //Start scene widgets
    TipsPushButton *start_load_button = nullptr;
    TipsPushButton *start_config_button = nullptr;
    TipsPushButton *start_exit_button = nullptr;
    QLabel *start_button_frame = nullptr;
    QLabel *start_cat_image = nullptr;
    QLabel *start_chat_bubble = nullptr;
    QLabel *start_title = nullptr;
    QLabel *start_tips = nullptr;

    AnimeFileHandle *start_title_handler = nullptr;
    AnimeFileHandle *start_cat_handler = nullptr;
    QTimer *start_title_timer = nullptr;
    QTimer *start_cat_timer = nullptr;
    QTimer *start_cat_timer2 = nullptr;

    //Load scene widgets
    TipsPushButton *load_connect_button = nullptr;
    TipsPushButton *load_exit_button = nullptr;
    QLabel *load_frame = nullptr;

    //Config scene widgets
    TipsPushButton *config_import_button = nullptr;
    TipsPushButton *config_revert_button = nullptr;
    TipsPushButton *config_delete_button = nullptr;
    TipsPushButton *config_exit_button = nullptr;
    QLabel *config_frame = nullptr;

    //shared widget
    QListWidget *shared_projects_list = nullptr;
    QTextEdit *shared_project_info = nullptr;
    QLabel *shared_project_img = nullptr;
    QLabel *shared_project_name = nullptr;
    QLabel *shared_project_tag = nullptr;
    QLabel *shared_project_tip = nullptr;

    //Connect Scene widgets
    TipsPushButton *connect_exit_button = nullptr;
    QLabel *connect_cat_image = nullptr;
    QLabel *connect_86gin_image = nullptr;
    QLabel *connect_cup_image1 = nullptr;
    QLabel *connect_cup_image2 = nullptr;
    QLabel *connect_link_image1 = nullptr;
    QLabel *connect_link_image2 = nullptr;
    QLabel *connect_wave_image1 = nullptr;
    QLabel *connect_wave_image2 = nullptr;
    QLabel *connect_finish_image = nullptr;
    QLabel *connect_chat_image1 = nullptr;
    QLabel *connect_chat_image2 = nullptr;
    QLabel *connect_chat_box1 = nullptr;
    QLabel *connect_chat_box2 = nullptr;
    QLabel *connect_ip_box = nullptr;
    QLabel *connect_speed_box1 = nullptr;
    QLabel *connect_speed_box2 = nullptr;
    QLabel *connect_speed_box3 = nullptr;
    QLabel *connect_speed_box4 = nullptr;

    AnimeFileHandle *connect_cat_handler = nullptr;
    AnimeFileHandle *connect_86gin_handler = nullptr;
    QTimer *connect_cat_timer = nullptr;
    QTimer *connect_cat_timer2 = nullptr;
    QTimer *connect_86gin_timer = nullptr;
    QTimer *connect_waves_timer1 = nullptr;
    QTimer *connect_waves_timer2 = nullptr;
    QTimer *connect_chat_timer1 = nullptr;
    QTimer *connect_chat_timer2 = nullptr;

    QString *connect_chat_str1 = nullptr;
    QString *connect_chat_str2 = nullptr;
    int connect_S2P;
    int connect_P2B;
    int connect_B2P;
    int connect_P2S;

    void connectChatAnime(QString* str1, QString* str2, bool order = true);

    //Layout methods
    void initStartPageLayout();
    void initLoadPageLayout();
    void initConfigPageLayout();
    void initShareWidget();
    void initConnectPageLayout();
    void startWidgetVisible(bool visible);
    void loadWidgetVisible(bool visible);
    void configWidgetVisible(bool visible);
    void connectWidgetVisible(bool visible);
    void connectLinkStasus(bool visible);

    //Change Scene objects
    bool change_scene_left = false;
    QTimer *change_scene_timer = nullptr;
    QString *change_scene_anime_start_bg = nullptr;
    QString *change_scene_anime_end_bg = nullptr;
    void (MainWindow::*change_scene_end_operation)() = nullptr;

    //Change Scene methods
    void doStartToLoadEnd();
    void doStartToConfigEnd();
    void doConnectDialog();
    void doLoadToConnect();
    void doLoadToConnectEnd();
    void doLoadToStartEnd();
    void doConfigToStartEnd();
    void doConnectToLoad();
    void doConnectToLoadEnd();

private slots:
    // Start Page
    void start_load_button_enter();
    void start_load_button_leave();
    void start_load_button_released();

    void start_config_button_enter();
    void start_config_button_leave();
    void start_config_button_released();

    void start_exit_button_enter();
    void start_exit_button_leave();
    void start_exit_button_released();

    void start_title_update();
    void start_cat_update();
    void start_cat_update2();

    // Load Page
    void load_connect_button_released();
    void load_connect_button_enter();
    void load_connect_button_leave();
    void load_exit_button_released();
    void load_exit_button_enter();
    void load_exit_button_leave();

    // Config Page
    void config_revert_button_released();
    void config_revert_button_enter();
    void config_revert_button_leave();

    void config_import_button_released();
    void config_import_button_enter();
    void config_import_button_leave();

    void config_delete_button_released();
    void config_delete_button_enter();
    void config_delete_button_leave();

    void config_exit_button_released();
    void config_exit_button_enter();
    void config_exit_button_leave();

    // Shared widge
    void do_load_project_list_current_row_changed(int);

    // Connect Dialog
    void connect_serial(QString);
    void connect_lan(QString, QString);

    // Connect Page
    void connect_exit_button_released();

    void connect_cat_update();
    void connect_cat_update2();
    void connect_86gin_update();
    void connect_waves1_update();
    void connect_waves2_update();
    void connect_chat1_update();
    void connect_chat2_update();

    void connect_helper_status(int);
    void connect_helper_speed(int, int, int, int);
    void connect_helper_IP(QString* );
    void connect_helper_error(int );


    // Change Scene Anime
    void change_scene_anime();

};

#endif // MAINWINDOW_H
