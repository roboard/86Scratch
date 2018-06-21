/* 
  mainwindow.cpp - DM&P 86Scratch
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

#include "mainwindow.h"
#include "tcp86Client.h"

#include <QCoreApplication>
#include <QFontDatabase>
#include <QMessageBox>
#include <QSettings>
#include <QFileDialog>
#include <QGraphicsDropShadowEffect>

#define cag(x) checkAndGetValue(x)
#define cagInt(x) checkAndGetValueInt(x)

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    //1. Set Work Directory
#if defined(Q_OS_MACOS)
    QDir::setCurrent(QCoreApplication::applicationDirPath() + "/..");
#else
    QDir::setCurrent(QCoreApplication::applicationDirPath());
#endif

    //2. Load Config.ini
    o_ini = new INIHandler();
    if(o_ini->loadIni("./config.ini") == false)
    {
        warnMessage("Can not found config.ini");
        init_fail = false;
        return;
    }

    // 3. Set Layout
    this->setWindowTitle("open-source-v1.1");
    this->setFixedSize(cagInt("WindowWidth"), cagInt("WindowHeight"));
    this->setWindowIcon(QIcon(cag("WindowIconPath")));
    setFont(cag("SystemFont"));

    initStartPageLayout();
    drawBG(cag("StartSceneBG"));
    startWidgetVisible(true);


    initLoadPageLayout();
    initConfigPageLayout();
    initShareWidget();
    loadWidgetVisible(false);
    configWidgetVisible(false);
    initConnectPageLayout();
    connectWidgetVisible(false);

    // 4.Init other settings
    change_scene_timer = new QTimer(this);
    connect(change_scene_timer, SIGNAL(timeout()), this, SLOT(change_scene_anime()));

    // 5. Start Anime
    start_title_timer->start(cagInt("start_title_ms"));
    start_cat_timer->start(cagInt("StartCatAnimeBlinkMs"));
}

MainWindow::~MainWindow()
{

}

/*
 * Overriding bool show()
 */
bool MainWindow::show()
{
   if(!init_fail)
       return false;
    QMainWindow::show();
    return true;
}

// System methods
/*
 * check and get value via key
 */
QString MainWindow::checkAndGetValue(QString key)
{
    int ok = o_ini->GET_SUCCESS;
    QString tmp;

    tmp = o_ini->checkAndGetValue(key, &ok);
    if(ok == o_ini->GET_NOKEY)
    {
        iniErrorMessage(key);
        return "";
    }

    return tmp;
}

/*
 * check and get int via key
 */
int MainWindow::checkAndGetValueInt(QString key)
{
    int ok = o_ini->GET_SUCCESS;
    int tmp;
    tmp = o_ini->checkAndGetValueInt(key, &ok);

    if(ok == o_ini->GET_NOKEY)
    {
        iniErrorMessage(key);
        return 0;
    }
    else if(ok == o_ini->GET_NOTINT)
    {
        iniIntErrorMessage(key);
        return 0;
    }

    return tmp;
}

/*
 * Draw QMainWindow Background
 */
void MainWindow::drawBG(QString file)
{
    QPixmap bkgnd(file);
    bkgnd = bkgnd.scaled(this->size(), Qt::IgnoreAspectRatio);
    QPalette palette;
    palette.setBrush(QPalette::Background, bkgnd);
    this->setPalette(palette);
}

/*
 * Check if python helper exists then copy to work directory
 */
bool MainWindow::helperCopy()
{
    //1. check if pythan helper exist
    QString project = shared_projects_list->currentItem()->text();

    QFile helper("./Projects/" + project + "/s2a_fm.py");
    QFile helper2("./Projects/" + project + "/scratch_command_handlers.py");
    QFile helper3("./Projects/" + project + "/scratch_http_server.py");

    if(!helper.exists() || !helper2.exists() || !helper3.exists())
    {
        warnMessage(cag("helperNotExists"));
        return false;
    }

    //2. Copy files to work directory
    QDir newDir("./Helper");
    if(QFile::exists(newDir.absolutePath() + "/s2a_fm.py"))
    {
        QFile::remove(newDir.absolutePath() + "/s2a_fm.py");
    }
    helper.copy(newDir.absolutePath() + "/s2a_fm.py");
    if(QFile::exists(newDir.absolutePath() + "/scratch_command_handlers.py"))
    {
        QFile::remove(newDir.absolutePath() + "/scratch_command_handlers.py");
    }
    helper2.copy(newDir.absolutePath() + "/scratch_command_handlers.py");
    if(QFile::exists(newDir.absolutePath() + "/scratch_http_server.py"))
    {
        QFile::remove(newDir.absolutePath() + "/scratch_http_server.py");
    }
    helper3.copy(newDir.absolutePath() + "/scratch_http_server.py");

    return true;
}

/*
 * Show Error Message about key not found then close app
 */
void MainWindow::iniErrorMessage(QString key)
{
    QMessageBox warningBox;
    warningBox.setText("Can not find " + key + " in config.ini");
    warningBox.exec();
    this->close();
}

/*
 * Show Error Message about value not available then close app
 */
void MainWindow::iniIntErrorMessage(QString key)
{
    QMessageBox warningBox;
    warningBox.setText("Ini parse failed. " + key + " should be an integer value");
    warningBox.exec();
    this->close();
}

/*
 * Load Font file
 */
void MainWindow::setFont(QString file)
{
    int id = QFontDatabase::addApplicationFont(cag("SystemFont"));
    if(id == -1)
    {
        warnMessage("Font file \"" + file + "\" can not be loaded");
        this->close();
    }
    QString family = QFontDatabase::applicationFontFamilies(id).at(0);
    o_font = new QFont(family);
}

/*
 * Default Project Name
 */
QString MainWindow::defaultProjectName(QString defaultBase)
{
    int serialNum = 1;
    QDir projects("./Projects");
    projects.setFilter(QDir::Dirs | QDir::NoDotAndDotDot);

    QStringList projectsList = projects.entryList();

    while(1){
        if(projectsList.contains(defaultBase + QString::number(serialNum)))
            serialNum++;
        else
            break;
    }

    return defaultBase + QString::number(serialNum);
}

/*
 * If theirs this project already
 */
bool MainWindow::containedProject(QString name)
{
    QDir projects("./Projects");
    projects.setFilter(QDir::Dirs | QDir::NoDotAndDotDot);

    QStringList projectList = projects.entryList();

    return projectList.contains(name);
}

/*
 * Copy files
 */
bool MainWindow::copyDirRecursively(QString sourceFolder, QString destFolder)
{
    bool success = false;
    QDir sourceDir(sourceFolder);

    if(!sourceDir.exists())
        return false;

    QDir destDir(destFolder);
    if(!destDir.exists())
        destDir.mkdir(destFolder);

    QStringList files = sourceDir.entryList(QDir::Files);
    for(QString file : files)
    {
        QString srcName = sourceFolder + QDir::separator() + file;
        QString destName = destFolder + QDir::separator() + file;
        success = QFile::copy(srcName, destName);
        if(!success)
            return false;
    }

    files.clear();
    files = sourceDir.entryList(QDir::AllDirs | QDir::NoDotAndDotDot);
    for(QString dir : files)
    {
        QString srcName = sourceFolder + QDir::separator() + dir;
        QString destName = destFolder + QDir::separator() + dir;
        success = copyDirRecursively(srcName, destName);
        if(success == false)
            return false;
    }

    return true;
}

/*
 * Update shared project list
 */
void MainWindow::updateProjectList()
{
    QDir projectsDir("./projects");
    projectsDir.refresh();
    projectsDir.setFilter(QDir::Dirs | QDir::NoDotAndDotDot);
    QStringList projectName = projectsDir.entryList();

    //1. add 86Scratch first
    int i = 0;
    if(projectName.contains("86Scratch"))
    {
        shared_projects_list->addItem("86Scratch");
        shared_projects_list->item(i++)->setSizeHint(QSize(cagInt("contain_width"), cagInt("contain_height")));
    }

    //2. add other project
    for(QString name : projectName)
    {
        if(name == "86Scratch")
            continue;

        shared_projects_list->addItem(name);
        int w = measureTextWidth(name, cag("SystemFont"), cagInt("contain_font_size"));
        if(w > cagInt("contain_width"))
            shared_projects_list->item(i++)->setSizeHint(QSize(w, cagInt("contain_height")));
        else
            shared_projects_list->item(i++)->setSizeHint(QSize(cagInt("contain_width"), cagInt("contain_height")));

   }
}

/*
 * Load project description
 */
void MainWindow::loadProjectData(int row)
{
    QString name = shared_projects_list->item(row)->text();
    QDir project("./Projects/" + name);
    QStringList filter("icon.*");
    QStringList files = project.entryList(filter);
    QString description("Description:\n");

    shared_project_name->setText(name);

    if(row == 0 && name == "86Scratch")
    {
        description += "  Default Project.";
        config_delete_button->setStyleSheet(cag("config_delete_button_disable_style_sheet"));
        config_delete_button->setEnabled(false);
    }
    else
    {
        config_delete_button->setEnabled(true);
        config_delete_button->setStyleSheet(cag("config_delete_button_style_sheet"));

        QFile file("./Projects/" + name + "/readme.txt");
        if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            description += " Can not found readme.txt.";
        }
        else
        {
            QString str(file.readAll());
            description += str;
        }
    }


    shared_project_info->setText(description);

    QPixmap *img;
    if(files.size() == 0)
    {
        //load default
        img = new QPixmap(cag("shared_projects_default_img"));
    }
    else
    {
        for(QString f : files)
        {
            img = new QPixmap("./projects/" + name + "/" + f);
            if(img != nullptr)
                break;
        }
    }
    *img = img->scaled(QSize(cagInt("shared_projects_img_width"), cagInt("shared_projects_img_height")));
    shared_project_img->setPixmap(*img);

    delete img;
}

/*
 * Launch Scratch & Load SB2 file
 */
void MainWindow::launchScratch(const QDir& sb2, const QStringList& sb2Files)
{
    QString sb2File = sb2.absolutePath() + "/" + sb2Files.at(0);
    scratch = new QProcess();
    scratch->setWorkingDirectory(sb2.absolutePath());
#if defined(Q_OS_LINUX) && defined(Q_PROCESSOR_X86_32)
#elif defined(Q_OS_LINUX) && defined(Q_PROCESSOR_X86_64)
#elif defined(Q_OS_MACOS)
    QString scratchPath = cag("MACOS_SCRATCH2_PATH");
    QFile scratchApp(scratchPath);
    if (!scratchApp.exists())
    {
        warnMessage(cag("macNoScratch2"));
        return;
    }
    scratch->start(scratchPath, QStringList() << sb2File);
#elif defined(Q_OS_WIN)
    QSettings m(cag("WINDOWS_SCRATCH2_PATH"), QSettings::NativeFormat);
    QStringList softwares = m.childGroups();
    for (const QString &software : softwares)
    {
        m.beginGroup(software);
        if (m.value("DisplayName").toString() == "Scratch 2 Offline Editor")
        {
            QString path = m.value("installLocation").toString();
            scratch->start(path + "/Scratch 2.exe", QStringList() << sb2File);
            return;
        }
        m.endGroup();
    }
    warnMessage(cag("winNoScratch2"));
#endif
}

/*
 * measure String view width
 */
int MainWindow::measureTextWidth(QString str, QString font_path, int font_size)
{
    QFont tmp(font_path, font_size);

    QFontMetrics fm(tmp);
    return fm.width(str);
}

/*
 * 86Scratch & 86duino chat anime
 */
void MainWindow::connectChatAnime(QString *str1, QString *str2, bool order){
    connect_chat_str1 = str1;
    connect_chat_str2 = str2;
    connect_chat_timer1->stop();
    connect_chat_timer2->stop();
    if(order){
        connect_chat_box1->setText(QString("_"));
        connect_chat_box2->setText(QString(""));
        connect_chat_timer1->start(cagInt("ConnectChatBoxTimer"));
    }else{
        connect_chat_box1->setText(QString(""));
        connect_chat_box2->setText(QString("_"));
        connect_chat_timer2->start(cagInt("ConnectChatBoxTimer"));
    }
}

/*
 * Create Warn Message Box
 */
void MainWindow::warnMessage(QString text)
{
    QMessageBox warningBox(this);
    warningBox.setText(text);
    warningBox.exec();
}



//Layout methods
/*
 * Init Start Page Widget
 */
void MainWindow::initStartPageLayout()
{

    //images
    start_button_frame = new QLabel(this);
    start_button_frame->setGeometry(cagInt("start_button_frame_x"), cagInt("start_button_frame_y"), cagInt("start_button_frame_width"), cagInt("start_button_frame_height"));
    start_button_frame->setPixmap(QPixmap(cag("start_button_frame_path")));

    start_cat_image = new QLabel(this);
    start_cat_image->setGeometry(cagInt("start_cat_image_x"), cagInt("start_cat_image_y"), cagInt("start_cat_image_width"), cagInt("start_cat_image_height"));

    start_chat_bubble = new QLabel(this);
    start_chat_bubble->setGeometry(cagInt("start_chat_bubble_x"), cagInt("start_chat_bubble_y"), cagInt("start_chat_bubble_width"), cagInt("start_chat_bubble_height"));
    start_chat_bubble->setPixmap(QPixmap(cag("start_chat_bubble")));

    start_title = new QLabel(this);
    start_title->setGeometry(cagInt("start_title_x"), cagInt("start_title_y"), cagInt("start_title_width"), cagInt("start_title_height"));

    //labels
    start_tips = new QLabel(this);
    start_tips->setText(cag("start_tips_label_init_text"));
    start_tips->setGeometry(cagInt("start_tips_label_x"), cagInt("start_tips_label_y"), cagInt("start_tips_label_width"), cagInt("start_tips_label_height"));
    start_tips->setWordWrap(true);
    start_tips->setStyleSheet(cag("start_tips_label_style_sheet"));
    start_tips->setFont(*o_font);

    //buttons
    start_load_button = new TipsPushButton(this);
    start_load_button->setGeometry(cagInt("start_load_button_x"), cagInt("start_load_button_y"), cagInt("start_load_button_width"), cagInt("start_load_button_height"));
    start_load_button->setFont(*o_font);
    start_load_button->setStyleSheet(cag("start_load_button_style_sheet"));
    start_load_button->setText(cag("start_load_button_text"));
    start_load_button->setAutoDefault(true);
    connect(start_load_button, SIGNAL(enter()), this, SLOT(start_load_button_enter()));
    connect(start_load_button, SIGNAL(leave()), this, SLOT(start_load_button_leave()));
    connect(start_load_button, SIGNAL(clicked()), this, SLOT(start_load_button_released()));

    start_config_button = new TipsPushButton(this);
    start_config_button->setGeometry(cagInt("start_config_button_x"), cagInt("start_config_button_y"), cagInt("start_config_button_width"), cagInt("start_config_button_height"));
    start_config_button->setFont(*o_font);
    start_config_button->setStyleSheet(cag("start_config_button_style_sheet"));
    start_config_button->setText(cag("start_config_button_text"));
    start_config_button->setAutoDefault(true);
    connect(start_config_button, SIGNAL(enter()), this, SLOT(start_config_button_enter()));
    connect(start_config_button, SIGNAL(leave()), this, SLOT(start_config_button_leave()));
    connect(start_config_button, SIGNAL(clicked()), this, SLOT(start_config_button_released()));

    start_exit_button = new TipsPushButton(this);
    start_exit_button->setGeometry(cagInt("start_exit_button_x"), cagInt("start_exit_button_y"), cagInt("start_exit_button_width"), cagInt("start_exit_button_height"));
    start_exit_button->setFont(*o_font);
    start_exit_button->setStyleSheet(cag("start_exit_button_style_sheet"));
    start_exit_button->setText(cag("start_exit_button_text"));
    start_exit_button->setAutoDefault(true);
    connect(start_exit_button, SIGNAL(enter()), this, SLOT(start_exit_button_enter()));
    connect(start_exit_button, SIGNAL(leave()), this, SLOT(start_exit_button_leave()));
    connect(start_exit_button, SIGNAL(clicked()), this, SLOT(start_exit_button_released()));

    start_title_handler = new AnimeFileHandle(cag("StartTitleAnime"), false);
    start_title_handler->resetIndex();
    start_title_timer = new QTimer(this);
    connect(start_title_timer, SIGNAL(timeout()), this, SLOT(start_title_update()));

    start_cat_handler = new AnimeFileHandle(cag("StartCatAnime"), false);
    start_cat_handler->resetIndex();
    start_cat_timer = new QTimer(this);
    connect(start_cat_timer, SIGNAL(timeout()), this, SLOT(start_cat_update()));

    start_cat_timer2 = new QTimer(this);
    connect(start_cat_timer2, SIGNAL(timeout()), this, SLOT(start_cat_update2()));
}

/*
 * Init load Page Widget
 */
void MainWindow::initLoadPageLayout()
{

    load_frame = new QLabel(this);
    load_connect_button = new TipsPushButton(this);
    load_exit_button = new TipsPushButton(this);

    load_frame->setGeometry(cagInt("load_button_frame_x"), cagInt("load_button_frame_y"), cagInt("load_button_frame_width"), cagInt("load_button_frame_height"));
    load_frame->setPixmap(QPixmap(cag("load_button_frame_path")));

    load_connect_button->setGeometry(cagInt("load_connect_button_x"), cagInt("load_connect_button_y"), cagInt("load_connect_button_width"), cagInt("load_connect_button_height"));
    load_connect_button->setText(cag("load_connect_button_text"));
    load_connect_button->setFont(*o_font);
    load_connect_button->setStyleSheet(cag("load_connect_button_style_sheet"));
    load_connect_button->setAutoDefault(true);
    connect(load_connect_button, SIGNAL(clicked()), this, SLOT(load_connect_button_released()));
    connect(load_connect_button, SIGNAL(enter()), this, SLOT(load_connect_button_enter()));
    connect(load_connect_button, SIGNAL(leave()), this, SLOT(load_connect_button_leave()));

    load_exit_button->setGeometry(cagInt("load_exit_button_x"), cagInt("load_exit_button_y"), cagInt("load_exit_button_width"), cagInt("load_exit_button_height"));
    load_exit_button->setText(cag("load_exit_button_text"));
    load_exit_button->setFont(*o_font);
    load_exit_button->setStyleSheet(cag("load_exit_button_style_sheet"));
    load_exit_button->setAutoDefault(true);
    connect(load_exit_button, SIGNAL(clicked()), this, SLOT(load_exit_button_released()));
    connect(load_exit_button, SIGNAL(enter()), this, SLOT(load_exit_button_enter()));
    connect(load_exit_button, SIGNAL(leave()), this, SLOT(load_exit_button_leave()));
}

/*
 * Init Config page Widget
 */
void MainWindow::initConfigPageLayout()
{
    config_frame = new QLabel(this);
    config_import_button = new TipsPushButton(this);
    config_revert_button = new TipsPushButton(this);
    config_delete_button = new TipsPushButton(this);
    config_exit_button = new TipsPushButton(this);

    config_import_button->setGeometry(cagInt("config_import_button_x"), cagInt("config_import_button_y"), cagInt("config_import_button_width"), cagInt("config_import_button_height"));
    config_revert_button->setGeometry(cagInt("config_revert_button_x"), cagInt("config_revert_button_y"), cagInt("config_revert_button_width"), cagInt("config_revert_button_height"));
    config_delete_button->setGeometry(cagInt("config_delete_button_x"), cagInt("config_delete_button_y"), cagInt("config_delete_button_width"), cagInt("config_delete_button_height"));
    config_exit_button->setGeometry(cagInt("config_exit_button_x"), cagInt("config_exit_button_y"), cagInt("config_exit_button_width"), cagInt("config_exit_button_height"));
    config_frame->setGeometry(cagInt("config_frame_button_x"), cagInt("config_frame_button_y"), cagInt("config_frame_button_width"), cagInt("config_frame_button_height"));

    config_import_button->setFont(*o_font);
    config_revert_button->setFont(*o_font);
    config_delete_button->setFont(*o_font);
    config_exit_button->setFont(*o_font);

    config_import_button->setStyleSheet(cag("config_import_button_style_sheet"));
    config_revert_button->setStyleSheet(cag("config_revert_button_style_sheet"));
    config_delete_button->setStyleSheet(cag("config_delete_button_style_sheet"));
    config_exit_button->setStyleSheet(cag("config_exit_button_style_sheet"));

    config_import_button->setText(cag("config_import_button_text"));
    config_revert_button->setText(cag("config_revert_button_text"));
    config_delete_button->setText(cag("config_delete_button_text"));
    config_exit_button->setText(cag("config_exit_button_text"));
    config_frame->setPixmap(QPixmap(cag("config_frame_button_path")));

    config_import_button->setAutoDefault(true);
    config_revert_button->setAutoDefault(true);
    config_delete_button->setAutoDefault(true);
    config_exit_button->setAutoDefault(true);

    connect(config_revert_button, SIGNAL(clicked()), this, SLOT(config_revert_button_released()));
    connect(config_revert_button, SIGNAL(enter()), this, SLOT(config_revert_button_enter()));
    connect(config_revert_button, SIGNAL(leave()), this, SLOT(config_revert_button_leave()));

    connect(config_import_button, SIGNAL(clicked()), this, SLOT(config_import_button_released()));
    connect(config_import_button, SIGNAL(enter()), this, SLOT(config_import_button_enter()));
    connect(config_import_button, SIGNAL(leave()), this, SLOT(config_import_button_leave()));

    connect(config_delete_button, SIGNAL(clicked()), this, SLOT(config_delete_button_released()));
    connect(config_delete_button, SIGNAL(enter()), this, SLOT(config_delete_button_enter()));
    connect(config_delete_button, SIGNAL(leave()), this, SLOT(config_delete_button_leave()));

    connect(config_exit_button, SIGNAL(clicked()), this, SLOT(config_exit_button_released()));
    connect(config_exit_button, SIGNAL(enter()), this, SLOT(config_exit_button_enter()));
    connect(config_exit_button, SIGNAL(leave()), this, SLOT(config_exit_button_leave()));

}

/*
 * Init Share Widget
 */
void MainWindow::initShareWidget()
{

    shared_projects_list = new QListWidget(this);
    shared_project_img = new QLabel(this);
    shared_project_name = new QLabel(this);
    shared_project_info = new QTextEdit(this);
    shared_project_tag = new QLabel(this);
    shared_project_tip = new QLabel(this);

    shared_projects_list->setGeometry(cagInt("shared_projects_list_x"), cagInt("shared_projects_list_y"), cagInt("shared_projects_list_width"), cagInt("shared_projects_list_height"));
    shared_project_img->setGeometry(cagInt("shared_projects_img_x"), cagInt("shared_projects_img_y"), cagInt("shared_projects_img_width"), cagInt("shared_projects_img_height"));
    shared_project_name->setGeometry(cagInt("shared_projects_name_x"), cagInt("shared_projects_name_y"), cagInt("shared_projects_name_width"), cagInt("shared_projects_name_height"));
    shared_project_info->setGeometry(cagInt("shared_projects_info_x"), cagInt("shared_projects_info_y"), cagInt("shared_projects_info_width"), cagInt("shared_projects_info_height"));
    shared_project_tag->setGeometry(cagInt("shared_projects_tag_x"), cagInt("shared_projects_tag_y"), cagInt("shared_projects_tag_width"), cagInt("shared_projects_tag_height"));
    shared_project_tip->setGeometry(cagInt("shared_projects_tip_x"), cagInt("shared_projects_tip_y"), cagInt("shared_projects_tip_width"), cagInt("shared_projects_tip_height"));

    shared_project_info->setReadOnly(true);

    shared_project_name->setFont(*o_font);
    shared_project_info->setFont(*o_font);
    shared_project_tag->setFont(*o_font);
    shared_project_tip->setFont(*o_font);
    shared_projects_list->setFont(*o_font);

    shared_projects_list->setStyleSheet(cag("shared_projects_list_style_sheet"));
    shared_project_name->setStyleSheet(cag("shared_projects_name_style_sheet"));
    shared_project_info->setStyleSheet(cag("shared_projects_info_style_sheet"));
    shared_project_tag->setStyleSheet(cag("shared_projects_tag_style_sheet"));
    QGraphicsDropShadowEffect* tagShadowEffect = new QGraphicsDropShadowEffect(this);
    tagShadowEffect->setXOffset(cagInt("shared_projects_tag_shadow_effect_x"));
    tagShadowEffect->setYOffset(cagInt("shared_projects_tag_shadow_effect_y"));
    shared_project_tag->setGraphicsEffect(tagShadowEffect);

    shared_project_tip->setStyleSheet(cag("shared_projects_tip_style_sheet"));

    shared_project_tip->setWordWrap(true);

    shared_projects_list->setTextElideMode(Qt::ElideNone);
    shared_projects_list->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);

    shared_project_info->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    shared_project_info->viewport()->setAutoFillBackground(false);
    shared_project_info->setFrameStyle(QFrame::NoFrame);


    shared_project_tip->setText(cag("shared_project_tip_text"));

    connect(shared_projects_list, SIGNAL(currentRowChanged(int)), this, SLOT(do_load_project_list_current_row_changed(int)));

    updateProjectList();
    shared_projects_list->setCurrentRow(0);
    loadProjectData(0);
}

/*
 * Init Connect Page Widget
 */
void MainWindow::initConnectPageLayout()
{
    // widgets init

    connect_cat_image = new QLabel(this);
    connect_86gin_image = new QLabel(this);
    connect_speed_box1 = new QLabel(this);
    connect_speed_box2 = new QLabel(this);
    connect_speed_box3 = new QLabel(this);
    connect_speed_box4 = new QLabel(this);
    connect_cup_image2 = new QLabel(this);
    connect_link_image1 = new QLabel(this);
    connect_link_image2 = new QLabel(this);
    connect_cup_image1 = new QLabel(this);
    connect_wave_image1 = new QLabel(this);
    connect_wave_image2 = new QLabel(this);
    connect_chat_image1 = new QLabel(this);
    connect_chat_image2 = new QLabel(this);
    connect_chat_box1 = new QLabel(this);
    connect_chat_box2 = new QLabel(this);
    connect_finish_image = new QLabel(this);
    connect_ip_box = new QLabel(this);

    connect_exit_button = new TipsPushButton(this);
    connect_exit_button->setFont(*o_font);
    connect_exit_button->setGeometry(cagInt("connect_exit_button_x"), cagInt("connect_exit_button_y"), cagInt("connect_exit_button_width"), cagInt("connect_exit_button_height"));
    connect_exit_button->setStyleSheet(cag("connect_exit_button_style_sheet"));
    connect_exit_button->setText(QString(cag("connect_exit_button_text")));
    connect(connect_exit_button, SIGNAL(clicked()), this, SLOT(connect_exit_button_released()));


    connect_cat_image->setGeometry(cagInt("connect_cat_image_x"), cagInt("connect_cat_image_y"), cagInt("connect_cat_image_width"), cagInt("connect_cat_image_height"));
    connect_86gin_image->setGeometry(cagInt("connect_86gin_image_x"), cagInt("connect_86gin_image_y"), cagInt("connect_86gin_image_width"), cagInt("connect_86gin_image_height"));
    connect_cup_image2->setGeometry(cagInt("connect_cup_image2_x"), cagInt("connect_cup_image2_y"), cagInt("connect_cup_image2_width"), cagInt("connect_cup_image2_height"));
    connect_link_image1->setGeometry(cagInt("connect_link_image1_x"), cagInt("connect_link_image1_y"), cagInt("connect_link_image1_width"), cagInt("connect_link_image1_height"));
    connect_link_image2->setGeometry(cagInt("connect_link_image2_x"), cagInt("connect_link_image2_y"), cagInt("connect_link_image2_width"), cagInt("connect_link_image2_height"));
    connect_cup_image1->setGeometry(cagInt("connect_cup_image1_x"), cagInt("connect_cup_image1_y"), cagInt("connect_cup_image1_width"), cagInt("connect_cup_image1_height"));
    connect_wave_image1->setGeometry(cagInt("connect_wave_image1_x"), cagInt("connect_wave_image1_y"), cagInt("connect_wave_image1_width"), cagInt("connect_wave_image1_height"));
    connect_wave_image2->setGeometry(cagInt("connect_wave_image2_x"), cagInt("connect_wave_image2_y"), cagInt("connect_wave_image2_width"), cagInt("connect_wave_image2_height"));
    connect_chat_image1->setGeometry(cagInt("connect_chat_image1_x"), cagInt("connect_chat_image1_y"), cagInt("connect_chat_image1_width"), cagInt("connect_chat_image1_height"));
    connect_chat_image2->setGeometry(cagInt("connect_chat_image2_x"), cagInt("connect_chat_image2_y"), cagInt("connect_chat_image2_width"), cagInt("connect_chat_image2_height"));
    connect_chat_box1->setGeometry(cagInt("connect_chat_box1_x"), cagInt("connect_chat_box1_y"), cagInt("connect_chat_box1_width"), cagInt("connect_chat_box1_height"));
    connect_chat_box2->setGeometry(cagInt("connect_chat_box2_x"), cagInt("connect_chat_box2_y"), cagInt("connect_chat_box2_width"), cagInt("connect_chat_box2_height"));
    connect_finish_image->setGeometry(cagInt("connect_finish_image_x"), cagInt("connect_finish_image_y"), cagInt("connect_finish_image_width"), cagInt("connect_finish_image_height"));
    connect_ip_box->setGeometry(cagInt("connect_ip_box_x"), cagInt("connect_ip_box_y"), cagInt("connect_ip_box_width"), cagInt("connect_ip_box_height"));
    connect_speed_box1->setGeometry(cagInt("connect_speed_box1_x"), cagInt("connect_speed_box1_y"), cagInt("connect_speed_box1_width"), cagInt("connect_speed_box1_height"));
    connect_speed_box2->setGeometry(cagInt("connect_speed_box2_x"), cagInt("connect_speed_box2_y"), cagInt("connect_speed_box2_width"), cagInt("connect_speed_box2_height"));
    connect_speed_box3->setGeometry(cagInt("connect_speed_box3_x"), cagInt("connect_speed_box3_y"), cagInt("connect_speed_box3_width"), cagInt("connect_speed_box3_height"));
    connect_speed_box4->setGeometry(cagInt("connect_speed_box4_x"), cagInt("connect_speed_box4_y"), cagInt("connect_speed_box4_width"), cagInt("connect_speed_box4_height"));


    connect_chat_box1->setFont(*o_font);
    connect_chat_box2->setFont(*o_font);
    connect_ip_box->setFont(*o_font);
    connect_speed_box1->setFont(*o_font);
    connect_speed_box2->setFont(*o_font);
    connect_speed_box3->setFont(*o_font);
    connect_speed_box4->setFont(*o_font);

    connect_chat_box1->setStyleSheet(cag("connect_chat_box1_style_sheet"));
    connect_chat_box2->setStyleSheet(cag("connect_chat_box2_style_sheet"));
    connect_ip_box->setStyleSheet(cag("connect_ip_box_style_sheet"));
    connect_speed_box1->setStyleSheet(cag("connect_speed_box1_style_sheet"));
    connect_speed_box2->setStyleSheet(cag("connect_speed_box2_style_sheet"));
    connect_speed_box3->setStyleSheet(cag("connect_speed_box3_style_sheet"));
    connect_speed_box4->setStyleSheet(cag("connect_speed_box4_style_sheet"));

    connect_speed_box1->setText(cag("connect_speed_box1_text"));
    connect_speed_box2->setText(cag("connect_speed_box2_text"));
    connect_speed_box3->setText(cag("connect_speed_box3_text"));
    connect_speed_box4->setText(cag("connect_speed_box4_text"));

    connect_chat_box1->setWordWrap(true);
    connect_chat_box2->setWordWrap(true);

    connect_link_image1->setPixmap(QPixmap(cag("connect_link_image1_img")));
    connect_link_image2->setPixmap(QPixmap(cag("connect_link_image2_img")));
    connect_cup_image1->setPixmap(QPixmap(cag("connect_cup_image1_img")));
    connect_cup_image2->setPixmap(QPixmap(cag("connect_cup_image2_img")));
    connect_wave_image1->setPixmap(QPixmap(cag("connect_wave_image1_img")));
    connect_wave_image2->setPixmap(QPixmap(cag("connect_wave_image2_img")));
    connect_chat_image1->setPixmap(QPixmap(cag("connect_chat_box1_img")));
    connect_chat_image2->setPixmap(QPixmap(cag("connect_chat_box2_img")));
    connect_finish_image->setPixmap(QPixmap(cag("connect_finish_image_img")));

    connect_cat_handler = new AnimeFileHandle(cag("connect_cat_image_directory"), false);
    connect_cat_handler->resetIndex();
    connect_86gin_handler = new AnimeFileHandle(cag("connect_86gin_image_directory"), true);
    connect_86gin_handler->resetIndex();

    connect_cat_timer = new QTimer(this);
    connect_cat_timer2 = new QTimer(this);
    connect_86gin_timer = new QTimer(this);
    connect_waves_timer1 = new QTimer(this);
    connect_waves_timer2 = new QTimer(this);
    connect_chat_timer1 = new QTimer(this);
    connect_chat_timer2 = new QTimer(this);
    connect(connect_cat_timer, SIGNAL(timeout()), this, SLOT(connect_cat_update()));
    connect(connect_cat_timer2, SIGNAL(timeout()), this, SLOT(connect_cat_update2()));
    connect(connect_86gin_timer, SIGNAL(timeout()), this, SLOT(connect_86gin_update()));
    connect(connect_waves_timer1, SIGNAL(timeout()), this, SLOT(connect_waves1_update()));
    connect(connect_waves_timer2, SIGNAL(timeout()), this, SLOT(connect_waves2_update()));
    connect(connect_chat_timer1, SIGNAL(timeout()), this, SLOT(connect_chat1_update()));
    connect(connect_chat_timer2, SIGNAL(timeout()), this, SLOT(connect_chat2_update()));

    client = new TCP86Client(this);

    connect(client, SIGNAL(helper_status(int)), this, SLOT(connect_helper_status(int)));
    connect(client, SIGNAL(helper_speed(int, int, int, int)), this, SLOT(connect_helper_speed(int, int, int, int)));
    connect(client, SIGNAL(helper_IP(QString*)), this, SLOT(connect_helper_IP(QString*)));
    connect(client, SIGNAL(helper_error(int)), this, SLOT(connect_helper_error(int)));

}


/*
 * Start page visible setting
 */
void MainWindow::startWidgetVisible(bool visible)
{
    if(start_exit_button == nullptr)
        return;
    start_exit_button->setVisible(visible);
    start_config_button->setVisible(visible);
    start_load_button->setVisible(visible);
    start_cat_image->setVisible(visible);
    start_chat_bubble->setVisible(visible);
    start_tips->setVisible(visible);
    start_title->setVisible(visible);
    start_button_frame->setVisible(visible);
    if(visible == false)
        start_title->clear();
}

/*
 * Load page visible setting
 */
void MainWindow::loadWidgetVisible(bool visible)
{
    // load scene only widgets
    if(load_frame == nullptr)
            return;
    load_frame->setVisible(visible);
    load_connect_button->setVisible(visible);
    load_exit_button->setVisible(visible);

    //shared widgets
    if(shared_projects_list == nullptr)
           return;
    shared_projects_list->setVisible(visible);
    shared_project_img->setVisible(visible);
    shared_project_name->setVisible(visible);
    shared_project_info->setVisible(visible);
    shared_project_tag->setVisible(visible);
    shared_project_tip->setVisible(visible);
    shared_project_tip->setText(cag("shared_project_tip_text"));
}

/*
 * Config page visible setting
 */
void MainWindow::configWidgetVisible(bool visible)
{
    if(config_import_button == nullptr)
        return;
    config_frame->setVisible(visible);

    config_import_button->setVisible(visible);
    config_revert_button->setVisible(visible);
    config_delete_button->setVisible(visible);
    config_exit_button->setVisible(visible);

    if(shared_projects_list == nullptr)
        return;
    shared_projects_list->setVisible(visible);
    shared_project_img->setVisible(visible);
    shared_project_name->setVisible(visible);
    shared_project_info->setVisible(visible);
    shared_project_tag->setVisible(visible);
    shared_project_tip->setVisible(visible);
    shared_project_tip->setText(cag("shared_project_tip_text"));
}

/*
 * Connect page visible setting
 */
void MainWindow::connectWidgetVisible(bool visible)
{
    connect_cat_image->setVisible(visible);
    connect_86gin_image->setVisible(visible);
    connect_exit_button->setVisible(visible);

    connect_cup_image1->setVisible(visible);
    connect_cup_image2->setVisible(visible);
    if(visible)
        connectLinkStasus(visible);
    else{
        connect_link_image1->setVisible(visible);
        connect_link_image2->setVisible(visible);
        connect_wave_image1->setVisible(visible);
        connect_wave_image2->setVisible(visible);
        connect_finish_image->setVisible(visible);
    }
    connect_chat_image1->setVisible(visible);
    connect_chat_image2->setVisible(visible);
    connect_chat_box1->setVisible(visible);
    connect_chat_box2->setVisible(visible);
    connect_ip_box->setVisible(visible);
    connect_speed_box1->setVisible(visible);
    connect_speed_box2->setVisible(visible);
    connect_speed_box3->setVisible(visible);
    connect_speed_box4->setVisible(visible);
}

/*
 * Link status widget visible setting
 */
void MainWindow::connectLinkStasus(bool visible)
{
    connect_link_image1->setVisible(visible);
    connect_link_image2->setVisible(!visible);
}

//Change Scene methods
/*
 * Start to Load page end
 */
void MainWindow::doStartToLoadEnd()
{
    shared_project_tag->setText(cag("start_load_button_text"));
    loadWidgetVisible(true);
}

/*
 * Start to config page end
 */
void MainWindow::doStartToConfigEnd()
{
    shared_project_tag->setText(cag("start_config_button_text"));
    configWidgetVisible(true);
}

/*
 * Show connect dialog
 */
void MainWindow::doConnectDialog()
{
    if(running == true)
        return;
    running = true;
    dialog_exception = false;

    dialog = new ConnectDialog(this);
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    dialog->setWindowTitle(cag("DialogTitle"));
    connect(dialog, SIGNAL(set_serial(QString)), this, SLOT(connect_serial(QString)));
    connect(dialog, SIGNAL(set_lan(QString,QString)), this, SLOT(connect_lan(QString,QString)));

    QString name = shared_projects_list->currentItem()->text();
    dialog->setName(name);
    dialog->exec();
}

/*
 *  load page to connect page
 */
void MainWindow::doLoadToConnect()
{
    change_scene_left = true;
    change_scene_anime_end_bg = new QString(cag("ConnectSceneBG"));
    change_scene_anime_start_bg = new QString(cag("LoadSceneBG"));
    change_scene_end_operation = doLoadToConnectEnd;
    qDebug() << parameterList;
    client->sessionOpened(parameterList);
    loadWidgetVisible(false);
    change_scene_timer->start(cagInt("ChangeSceneTotalMs")/cagInt("ChangeSceneFPS"));
}

/*
 * Load to Connect page end
 */
void MainWindow::doLoadToConnectEnd()
{
    connectWidgetVisible(true);
    connect_cat_timer->start(cagInt("ConnectCatTimer"));
    connect_86gin_timer->start(cagInt("Connect86ginTimer"));
    connect_waves_timer1->start(cagInt("ConnectWavesTimer1"));
}

/*
 * Load to Start page end
 */
void MainWindow::doLoadToStartEnd()
{
    startWidgetVisible(true);
    start_title_timer->start(cagInt("start_title_ms"));
    start_cat_timer->start(cagInt("StartCatAnimeBlinkMs"));
}

/*
 * Config to Start page end
 */
void MainWindow::doConfigToStartEnd()
{
    startWidgetVisible(true);
    start_title_timer->start(cagInt("start_title_ms"));
    start_cat_timer->start(cagInt("StartCatAnimeBlinkMs"));
}

/*
 * Connect to Load page
 */
void MainWindow::doConnectToLoad()
{
    parameterList.clear();
    change_scene_left = false;
    change_scene_anime_end_bg = new QString(cag("LoadSceneBG"));
    change_scene_anime_start_bg = new QString(cag("ConnectSceneBG"));
    change_scene_end_operation = doConnectToLoadEnd;

    connect_cat_timer->stop();
    connect_86gin_timer->stop();
    connect_waves_timer1->stop();
    connect_waves_timer2->stop();

    connectWidgetVisible(false);
    change_scene_timer->start(cagInt("ChangeSceneTotalMs")/cagInt("ChangeSceneFPS"));
}

/*
 * Connect to Load page End
 */
void MainWindow::doConnectToLoadEnd()
{
    loadWidgetVisible(true);
    running = 0;

}

// slots

/*
 * start page load button enter event
 */
void MainWindow::start_load_button_enter()
{
    start_tips->setText(cag("start_load_button_enter_text"));
}

/*
 * start page load button leave event
 */
void MainWindow::start_load_button_leave()
{
    start_tips->setText(cag("start_tips_label_init_text"));
}

/*
 * start page load button press handler
 */
void MainWindow::start_load_button_released()
{
    change_scene_left = true;
    change_scene_anime_end_bg = new QString(cag("LoadSceneBG"));
    change_scene_anime_start_bg = new QString(cag("StartSceneBG"));
    change_scene_end_operation = doStartToLoadEnd;

    start_title_timer->stop();
    start_cat_timer->stop();
    start_cat_timer2->stop();
    start_title_handler->resetIndex();
    start_cat_handler->resetIndex();

    startWidgetVisible(false);
    change_scene_timer->start(cagInt("ChangeSceneTotalMs")/cagInt("ChangeSceneFPS"));
}

/*
 * start page config button enter event
 */
void MainWindow::start_config_button_enter()
{
    start_tips->setText(QString(cag("start_config_button_enter_text")));
}

/*
 * start page config button leave event
 */
void MainWindow::start_config_button_leave()
{
    start_tips->setText(cag("start_tips_label_init_text"));
}

/*
 * start page config button press handler
 */
void MainWindow::start_config_button_released()
{
    change_scene_left = false;
    change_scene_anime_end_bg = new QString(cag("ConfigSceneBG"));
    change_scene_anime_start_bg = new QString(cag("StartSceneBG"));
    change_scene_end_operation = doStartToConfigEnd;

    start_title_timer->stop();
    start_cat_timer->stop();
    start_cat_timer2->stop();
    start_title_handler->resetIndex();
    start_cat_handler->resetIndex();

    startWidgetVisible(false);
    change_scene_timer->start(cagInt("ChangeSceneTotalMs")/cagInt("ChangeSceneFPS"));
}

/*
 * start page exit button enter event
 */
void MainWindow::start_exit_button_enter()
{
    start_tips->setText(QString(cag("start_exit_button_enter_text")));
}

/*
 * start page exit button leave event
 */
void MainWindow::start_exit_button_leave()
{
    start_tips->setText(cag("start_tips_label_init_text"));
}

/*
 * start page exit button press handler
 */
void MainWindow::start_exit_button_released()
{
    this->close();
}

/*
 * Start page title anime
 */
void MainWindow::start_title_update()
{
    QPixmap img(start_title_handler->getNextImagePath());
    start_title->setPixmap(img);

    if(start_title_handler->isEnd())
    {
        start_title_timer->stop();
        start_title_handler->resetIndex();
    }
}

/*
 * Start page Cat anime
 */
void MainWindow::start_cat_update()
{
    QPixmap img(start_cat_handler->getNextImagePath());
    start_cat_image->setPixmap(img);

    if(start_cat_handler->isEnd())
    {
        start_cat_timer->stop();
        start_cat_timer2->start(cagInt("StartCatAnimeRestartMs"));
        start_cat_handler->resetIndex();
    }
}

/*
 * Start Cat Repeat Controller
 */
void MainWindow::start_cat_update2()
{
    start_cat_timer2->stop();
    start_cat_timer->start(cagInt("StartCatAnimeBlinkMs"));
}

/*
 * Load page connect button release
 */
void MainWindow::load_connect_button_released()
{
    doConnectDialog();

    if(dialog_exception == false)
        doLoadToConnect();
    else
        running = false;

}

/*
 * Load page connect button enter event
 */
void MainWindow::load_connect_button_enter()
{
    shared_project_tip->setText(cag("load_connect_button_tip"));
}

/*
 * Load page connect button leave event
 */
void MainWindow::load_connect_button_leave()
{
    shared_project_tip->setText(cag("shared_project_tip_text"));
}

/*
 * Load page exit button release
 */
void MainWindow::load_exit_button_released()
{
    change_scene_left = false;
    change_scene_anime_end_bg = new QString(cag("StartSceneBG"));
    change_scene_anime_start_bg = new QString(cag("LoadSceneBG"));
    change_scene_end_operation = doLoadToStartEnd;

    loadWidgetVisible(false);
    change_scene_timer->start(cagInt("ChangeSceneTotalMs")/cagInt("ChangeSceneFPS"));
}

/*
 * Load page exit button enter event
 */
void MainWindow::load_exit_button_enter()
{
    shared_project_tip->setText(cag("load_exit_button_tip"));
}

/*
 * Load page exit button leave event
 */
void MainWindow::load_exit_button_leave()
{
    shared_project_tip->setText(cag("shared_project_tip_text"));
}


/*
 * Config Page Revert button release
 */
void MainWindow::config_revert_button_released()
{
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, cag("config_qa_title_revert"), cag("confiq_qa_question_revert"), QMessageBox::Yes|QMessageBox::No);

    if(reply == QMessageBox::No)
        return;

    QString name = shared_projects_list->currentItem()->text();
    QDir project("./Projects/" + name);
    QDir ori("./Projects/" + name + "/original");

    if(project.exists() && ori.exists())
    {
        QStringList filter("*.sb2");
        QStringList files = project.entryList(filter);
        for(QString filename : files)
        {
            if(project.remove(filename) == false)
            {
                warnMessage(cag("warn_caanot_remove"));
                return;
            }
        }

        files = ori.entryList(filter);
        for(QString filename : files)
        {
            QFile orgsb2(ori.absolutePath() + "/" + filename);
            if(orgsb2.copy(project.absolutePath() + "/" + filename) == false)
            {
                warnMessage(cag("warn_cannot_copy"));
                return;
            }
        }

        warnMessage(cag("configRevertOK"));
    }
    else
    {
        warnMessage(cag("configRevertFail"));
    }
}

/*
 * Config Page Revert button enter event
 */
void MainWindow::config_revert_button_enter()
{
    shared_project_tip->setText(cag("config_revert_button_tip"));
}

/*
 * Config Page Revert button leave event
 */
void MainWindow::config_revert_button_leave()
{
    shared_project_tip->setText(cag("shared_project_tip_text"));
}


/*
 * Config Page Import button release
 */
void MainWindow::config_import_button_released()
{
    QSettings setting;
    QString importPath = QFileDialog::getExistingDirectory(this, tr("Open Directory"), setting.value("default_dlr").toString(), QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

    if(!importPath.isEmpty())
    {
        QDir importDir(importPath);

        int pos = 0;
        QString Name = importDir.dirName();
        if(containedProject(Name))
            Name = defaultProjectName(Name);

        QDir projectDir;
        if(projectDir.mkdir("./Projects/" + Name) == false || projectDir.mkdir("./Projects/" + Name + "/original") == false)
        {
            warnMessage(cag("warn_cannot_mkdir"));
            return;
        }
        QDir newDir("./Projects/" + Name);
        QDir oriDir("./Projects/" + Name + "/original");
        QStringList filenames = importDir.entryList();
        for(QString filename : filenames)
        {
            if(filename == "." || filename == "..")
                continue;
            QFileInfo info(importDir.absolutePath() + "/" + filename);
            if(info.isFile())
            {
                QFile copiedFile(importDir.absolutePath() + "/" + filename);
                if(copiedFile.copy(newDir.absolutePath() + "/" + filename) == false)
                {
                    warnMessage(cag("warn_cannot_copy"));
                    return;
                }
                if(filename.contains(".sb2"))
                {
                    if(copiedFile.copy(oriDir.absolutePath() + "/" + filename) == false)
                    {
                        warnMessage(cag("warn_cannot_copy"));
                        return;
                    }
                }
            }
            else if(info.isDir())
            {
                if(copyDirRecursively(importDir.absolutePath() + "/" + filename, newDir.absolutePath() + "/" + filename) == false)
                {
                    warnMessage(cag("warn_cannot_copy"));
                    return;
                }
            }
        }
        shared_projects_list->addItem(Name);
        int w = measureTextWidth(Name, cag("SystemFont"), cagInt("contain_font_size"));
        if(w > cagInt("contain_width"))
            shared_projects_list->item(shared_projects_list->count() - 1)->setSizeHint(QSize(w, cagInt("contain_height")));
        else
            shared_projects_list->item(shared_projects_list->count() - 1)->setSizeHint(QSize(cagInt("contain_width"), cagInt("contain_height")));


        warnMessage(cag("configImportOK"));
    }
}

/*
 * Config Page Import button enter event
 */
void MainWindow::config_import_button_enter()
{
    shared_project_tip->setText(cag("config_import_button_tip"));
}

/*
 * Config Page Import button leave event
 */
void MainWindow::config_import_button_leave()
{
    shared_project_tip->setText(cag("shared_project_tip_text"));
}


/*
 * Config Page Delete button release
 */
void MainWindow::config_delete_button_released()
{
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, cag("config_qa_title_delete"), cag("config_qa_question_delete"), QMessageBox::Yes|QMessageBox::No);

    if(reply == QMessageBox::Yes)
    {
        QString name = shared_projects_list->currentItem()->text();
        QDir project("./Projects/" + name);
        if(project.removeRecursively() == false)
        {
            warnMessage(cag("warn_cannot_remove"));
            return;
        }
        //QDir out("./Projects");
        //out.remove(name);
        shared_projects_list->takeItem(shared_projects_list->currentRow());
        warnMessage(cag("configDeleteOK"));
    }
}

/*
 * Config Page Delete button enter event
 */
void MainWindow::config_delete_button_enter()
{
    shared_project_tip->setText(cag("config_delete_button_tip"));
}

/*
 * Config Page Delete button leave event
 */
void MainWindow::config_delete_button_leave()
{
    shared_project_tip->setText(cag("shared_project_tip_text"));
}


/*
 * Config Page Exit button release
 */
void MainWindow::config_exit_button_released()
{
    change_scene_left = true;
    change_scene_anime_end_bg = new QString(cag("StartSceneBG"));
    change_scene_anime_start_bg = new QString(cag("ConfigSceneBG"));
    change_scene_end_operation = doConfigToStartEnd;

    configWidgetVisible(false);
    change_scene_timer->start(cagInt("ChangeSceneTotalMs")/cagInt("ChangeSceneFPS"));
}

/*
 * Config Page Exit button enter event
 */
void MainWindow::config_exit_button_enter()
{
    shared_project_tip->setText(cag("config_exit_button_tip"));
}

/*
 * Config Page Exit button leave event
 */
void MainWindow::config_exit_button_leave()
{
    shared_project_tip->setText(cag("shared_project_tip_text"));
}


/*
 * Project List selected changed event
 */
void MainWindow::do_load_project_list_current_row_changed(int row)
{
    loadProjectData(row);
}

/*
 * Configure dialog emit set_serial
 */
void MainWindow::connect_serial(QString portName)
{
    dialog_exception = false;

    // 1. Check if port available.
    if(portName == "")
    {
        warnMessage(cag("serialNoAvailablePort"));
        dialog_exception = true;
        return;
    }

    // 2. check if pythan helper exist
    if(!helperCopy())
    {
        dialog_exception = true;
        return;
    }

    // 3. modify portName;
#if defined(Q_OS_LINUX) && defined(Q_PROCESSOR_X86_32)
    portName = "/dev/" + portName;
#elif defined(Q_OS_LINUX) && defined(Q_PROCESSOR_X86_64)
    portName = "/dev/" + portName;
#elif defined(Q_OS_MACOS)
    portName = "/dev/" + portName;
#elif defined(Q_OS_WIN)
    portName = portName;
#endif

    //4. save parameter
    portName = portName.toLower();
    parameterList << "s2a_fm.py" << portName;

    dialog_exception = false;
    return;
}

/*
 * Configure dialog emit set_lan
 */
void MainWindow::connect_lan(QString ip, QString port)
{
    dialog_exception = false;

    // 1. Check if IP & socket available
    if(ip == "" || port == "")
    {
        warnMessage(cag("lanNoAvailableIP"));
        dialog_exception = true;
        return;
    }

    // 2. check if pythan helper exist
    if(!helperCopy())
    {
        dialog_exception = true;
        return;
    }

    //3. save parameters
    if(ip != "autodetect")
        parameterList << "s2a_fm.py" << ip << port;
    else
        parameterList << "s2a_fm.py" << ip;

    dialog_exception = false;
    return;
}

/*
 * Connect Page Exit pressed
 */
void MainWindow::connect_exit_button_released()
{

    QMessageBox::StandardButton reply;
    if(connect_exit_button->text() == cag("connect_reconnect_button_text")){
        reply = QMessageBox::question(this, cag("connect_reconnect_button_text"), cag("connect_reconnect_messagebox_question"), QMessageBox::Yes|QMessageBox::No);

        if(reply == QMessageBox::Yes)
        {
            client->TCPdisconnect();
            client->sessionOpened(parameterList);
        }else{
            client->TCPdisconnect();
            doConnectToLoad();
        }
    }
    else
    {
        reply = QMessageBox::question(this, cag("connect_exit_button_text"), cag("connect_exit_messagebox_question"), QMessageBox::Yes|QMessageBox::No);

        if(reply == QMessageBox::Yes)
        {
            client->TCPdisconnect();
            doConnectToLoad();
        }
    }
}

/*
 * Connect Page Cat anime
 */
void MainWindow::connect_cat_update()
{
    QPixmap img(connect_cat_handler->getNextImagePath());
    connect_cat_image->setPixmap(img);

    if(connect_cat_handler->isEnd())
    {
        connect_cat_timer->stop();
        connect_cat_timer2->start(qrand() % 8000);
        connect_cat_handler->resetIndex();
    }
}

/*
 * Connect Page Cat anime restart timer
 */
void MainWindow::connect_cat_update2()
{
    connect_cat_timer2->stop();
    connect_cat_timer->start(cagInt("ConnectCatTimer"));
}

/*
 * Connect Page 86Gin Anime
 */
void MainWindow::connect_86gin_update()
{
    QPixmap img(connect_86gin_handler->getNextImagePath());
    connect_86gin_image->setPixmap(img);
}

/*
 * Connect Page Communication Wave
 */
void MainWindow::connect_waves1_update()
{
    connect_wave_image1->setVisible(true);
    int x = connect_wave_image1->pos().x() + 10;
    connect_wave_image1->move(x, 0);

    if(x < 100)
    {
        connect_speed_box1->setText(QString::number(connect_S2P) + " bps");
        if(connect_S2P > 1000)
            connect_speed_box1->setStyleSheet(cag("connect_speed_box1_style_sheet_slow"));
        else
            connect_speed_box1->setStyleSheet(cag("connect_speed_box1_style_sheet_fast"));
        connect_speed_box1->setVisible(connect_S2P > 0 && connect_S2P < 32767);
        connect_speed_box2->setVisible(false);
        connect_speed_box3->setVisible(false);
        connect_speed_box4->setVisible(false);
    }else{
        connect_speed_box2->setText(QString::number(connect_P2B) + " bps");
        if(connect_P2B > 1000)
            connect_speed_box2->setStyleSheet(cag("connect_speed_box2_style_sheet_slow"));
        else
            connect_speed_box2->setStyleSheet(cag("connect_speed_box2_style_sheet_fast"));
        connect_speed_box1->setVisible(false);
        connect_speed_box2->setVisible(connect_P2B > 0 && connect_P2B < 32767);
        connect_speed_box3->setVisible(false);
        connect_speed_box4->setVisible(false);
    }
    if(x > 200)
    {
        connect_wave_image1->setVisible(false);
        connect_wave_image1->setGeometry(0, 0, 800, 450);
        connect_waves_timer1->stop();
        connect_waves_timer2->start(cagInt("connect_wave_image2_speed_ms"));
    }
}

void MainWindow::connect_waves2_update()
{
    connect_wave_image2->setVisible(true);
    int x = connect_wave_image2->pos().x() - 10;
    connect_wave_image2->move(x, 0);

    if(x > -100)
    {
        connect_speed_box4->setText(QString::number(connect_P2S) + " bps");
        if(connect_P2S > 1000)
            connect_speed_box4->setStyleSheet(cag("connect_speed_box4_style_sheet_slow"));
        else
            connect_speed_box4->setStyleSheet(cag("connect_speed_box4_style_sheet_fast"));
        connect_speed_box1->setVisible(false);
        connect_speed_box2->setVisible(false);
        connect_speed_box3->setVisible(false);
        connect_speed_box4->setVisible(connect_P2S > 0 && connect_P2S < 32767);
    }else{
        connect_speed_box3->setText(QString::number(connect_B2P) + " bps");
        if(connect_B2P > 1000)
            connect_speed_box3->setStyleSheet(cag("connect_speed_box3_style_sheet_slow"));
        else
            connect_speed_box3->setStyleSheet(cag("connect_speed_box3_style_sheet_fast"));
        connect_speed_box1->setVisible(false);
        connect_speed_box2->setVisible(false);
        connect_speed_box3->setVisible(connect_B2P > 0 && connect_B2P < 32767);
        connect_speed_box4->setVisible(false);
    }
    if(x < -200)
    {
        connect_wave_image2->setVisible(false);
        connect_wave_image2->setGeometry(0, 0, 800, 450);
        connect_waves_timer2->stop();
        connect_waves_timer1->start(cagInt("connect_wave_image2_speed_ms"));
    }
}

/*
 * Connect Page chat content
 */
void MainWindow::connect_chat1_update()
{
    if(connect_chat_box1->text().length() <= connect_chat_str1->length()){
        connect_chat_box1->setText(connect_chat_str1->left(connect_chat_box1->text().length()) + "_");

        connect_cup_image1->move(cagInt("connect_cup_image1_x") + (connect_chat_box1->text().length() % 2) * 1, cagInt("connect_cup_image1_y") +(connect_chat_box1->text().length() % 2) * 1);
        connect_link_image1->move(cagInt("connect_link_image1_x"), cagInt("connect_link_image1_y")+(connect_chat_box1->text().length() % 2) * 1);
    }
    else
    {
        connect_cup_image1->move(cagInt("connect_cup_image1_x"), cagInt("connect_cup_image1_y"));
        connect_link_image1->move(cagInt("connect_link_image1_x"), cagInt("connect_link_image1_y"));
        if(connect_chat_box2->text().length() == connect_chat_str2->length() || connect_chat_str2->length() == 0){
            switch(connect_chat_box1->text().at(connect_chat_str1->length()).unicode()){
            case '_':
                connect_chat_box1->setText(*connect_chat_str1 + "-");
                break;
            case '-':
                connect_chat_box1->setText(*connect_chat_str1 + "/");
                break;
            case '/':
                connect_chat_box1->setText(*connect_chat_str1 + "|");
                break;
            case '|':
                connect_chat_box1->setText(*connect_chat_str1 + "\\");
                break;
            case '\\':
                connect_chat_box1->setText(*connect_chat_str1 + "-");
                break;
            }
        }
        else
        {
            connect_chat_box1->setText(*connect_chat_str1);
            connect_chat_timer1->stop();
            connect_chat_timer2->start(cagInt("ConnectChatBoxTimer"));
        }
    }
}

void MainWindow::connect_chat2_update()
{
    if(connect_chat_box2->text().length() <= connect_chat_str2->length()){
        connect_chat_box2->setText(connect_chat_str2->left(connect_chat_box2->text().length()) + "_");

        connect_cup_image2->move(cagInt("connect_cup_image2_x") + (connect_chat_box2->text().length() % 2) * 1, cagInt("connect_cup_image2_y") + (connect_chat_box2->text().length() % 2) * -1);
        connect_link_image1->move(cagInt("connect_link_image1_x"), cagInt("connect_link_image1_y") + (connect_chat_box2->text().length() % 2) * -1);
    }
    else
    {
        connect_cup_image2->move(cagInt("connect_cup_image2_x"), cagInt("connect_cup_image2_y"));
        connect_link_image1->move(cagInt("connect_link_image1_x"), cagInt("connect_link_image1_y"));
        if(connect_chat_box1->text().length() == connect_chat_str1->length() || connect_chat_str1->length() == 0){
            switch(connect_chat_box2->text().at(connect_chat_str2->length()).unicode()){
            case '_':
                connect_chat_box2->setText(*connect_chat_str2 + "-");
                break;
            case '-':
                connect_chat_box2->setText(*connect_chat_str2 + "/");
                break;
            case '/':
                connect_chat_box2->setText(*connect_chat_str2 + "|");
                break;
            case '|':
                connect_chat_box2->setText(*connect_chat_str2 + "\\");
                break;
            case '\\':
                connect_chat_box2->setText(*connect_chat_str2 + "-");
                break;
            }
        }
        else
        {
            connect_chat_box2->setText(*connect_chat_str2);
            connect_chat_timer2->stop();
            connect_chat_timer1->start(cagInt("ConnectChatBoxTimer"));
        }
    }
}

/*
 * Ask pyhon_helper for status
 */
void MainWindow::connect_helper_status(int status){
    if(client->connect_status != client->finish_status){
        connect_finish_image->setVisible(false);
    }
    if(client->connect_status != status){
        QString *str1;
        QString *str2;
        QString key = cag("order_key");
        client->connect_status = status;
        client->status_Show(status, str1, str2);
        connectChatAnime(&(str1->replace(key,"")),&(str2->replace(key,"")),str2->at(0) != key);

        if(status == client->finish_status)
        {
            connect_finish_image->setVisible(true);

            if(connect_waves_timer1->isActive() == false)
            {
                connect_waves_timer1->start(cagInt("ConnectWavesTimer1"));
                connect_wave_image1->move(0,0);
                connect_wave_image2->move(0,0);
            }


            connect_speed_box1->setVisible(true);
            connect_speed_box2->setVisible(true);
            connect_speed_box3->setVisible(true);
            connect_speed_box4->setVisible(true);
            connect_wave_image1->setVisible(true);
            connect_wave_image2->setVisible(true);
            connect_link_image1->setVisible(true);
            connect_link_image2->setVisible(false);
            client->connect_error = 0x00;

        }
        else if(status == (client->finish_status - 1))
        {

            QString project = shared_projects_list->currentItem()->text();
            QDir sb2("./Projects/" + project);
            QStringList filter("*.sb2");
            QStringList sb2Files = sb2.entryList(filter);
            if (sb2Files.count() == 0)
            {
                warnMessage(cag("noSb2File"));
                return;
            }
            launchScratch(sb2, sb2Files);
        }
        connect_exit_button->setStyleSheet(cag("connect_exit_button_style_sheet"));
        connect_exit_button->setText(QString(cag("connect_exit_button_text")));
    }
}

/*
 * Transmit speed between Scratch & 86Duino
 */
void MainWindow::connect_helper_speed(int S2P, int P2B, int B2P, int P2S){
    connect_S2P = S2P;
    connect_P2B = P2B;
    connect_B2P = B2P;
    connect_P2S = P2S;
}

/*
 * 86Duino IP (If using network)
 */
void MainWindow::connect_helper_IP(QString* IP_Addr){
    if(*IP_Addr != "0.0.0.0")
    {
        connect_ip_box->setVisible(true);
        connect_ip_box->setText(*IP_Addr);
    }
    else
        connect_ip_box->setVisible(false);
}

/*
 * pyhon_helper error code
 */
void MainWindow::connect_helper_error(int errorcode){
    if(client->connect_error != errorcode){
        QString *str1;
        QString *str2;
        QString key = cag("order_key");
        client->connect_error = errorcode;
        client->errorcode_Show(errorcode, str1, str2);
        connectChatAnime(&(str1->replace(key,"")),&(str2->replace(key,"")),str2->at(0) != key);

        connect_waves_timer1->stop();
        connect_waves_timer2->stop();

        connect_speed_box1->setVisible(false);
        connect_speed_box2->setVisible(false);
        connect_speed_box3->setVisible(false);
        connect_speed_box4->setVisible(false);
        connect_wave_image1->setVisible(false);
        connect_wave_image2->setVisible(false);
        connect_link_image1->setVisible(false);
        connect_link_image2->setVisible(true);
        connect_exit_button->setStyleSheet(cag("connect_reconnect_button_style_sheet"));
        connect_exit_button->setText(QString(cag("connect_reconnect_button_text")));
    }
}

/*
 * Change Scene Anime Controller
 */
void MainWindow::change_scene_anime()
{
    static int count = 0;
    static int pieces = 0;
    static int total_count = 0;
    static QLabel *left = nullptr;
    static QLabel *right = nullptr;

    if(left == nullptr && right == nullptr)
    {
        if(change_scene_anime_start_bg == nullptr || change_scene_anime_end_bg == nullptr)
        {
            change_scene_timer->stop();
            return;
        }

        QImage start_img(*change_scene_anime_start_bg);
        start_img = start_img.scaled(this->size(), Qt::IgnoreAspectRatio);
        QImage end_img(*change_scene_anime_end_bg);
        end_img = end_img.scaled(this->size(), Qt::IgnoreAspectRatio);

        if(change_scene_left)
        {
            left = new QLabel(this);
            left->setPixmap(QPixmap::fromImage(start_img));
            left->setGeometry(0, 0, cagInt("WindowWidth"), cagInt("WindowHeight"));

            right = new QLabel(this);
            right->setPixmap(QPixmap::fromImage(end_img));
            right->setGeometry(cagInt("WindowWidth"), 0, cagInt("WindowWidth"), cagInt("WindowHeight"));
        }
        else
        {
            right = new QLabel(this);
            right->setPixmap(QPixmap::fromImage(start_img));
            right->setGeometry(0, 0, cagInt("WindowWidth"), cagInt("WindowHeight"));

            left = new QLabel(this);
            left->setPixmap(QPixmap::fromImage(end_img));
            left->setGeometry(-1 * cagInt("WindowWidth"), 0, cagInt("WindowWidth"), cagInt("WindowHeight"));
        }
        total_count = cagInt("ChangeSceneTotalMs") / cagInt("ChangeSceneFPS");
        pieces = cagInt("WindowWidth") / total_count;
        right->show();
        left->show();
    }

    if(change_scene_left)
    {
        left->move(0 - count * pieces, 0);
        right->move(cagInt("WindowWidth") - count * pieces, 0);
    }
    else
    {
        left->move(-1 * cagInt("WindowWidth") + count * pieces, 0);
        right->move(0 + count * pieces, 0);
    }
    count++;

    if(count == total_count)
    {
        left->setVisible(false);
        right->setVisible(false);

        change_scene_timer->stop();

        if(change_scene_end_operation != nullptr)
        {
            (this->*change_scene_end_operation)();
        }

        drawBG(*change_scene_anime_end_bg);

        delete change_scene_anime_start_bg;
        delete change_scene_anime_end_bg;
        change_scene_anime_start_bg = nullptr;
        change_scene_anime_end_bg = nullptr;

        delete left;
        delete right;
        left = nullptr;
        right = nullptr;
        count = 0;
        total_count = 0;
        pieces = 0;
    }
}

