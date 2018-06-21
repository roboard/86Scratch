/* 
  inihandler.cpp - DM&P 86Scratch
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

#include "inihandler.h"

#include <QFile>

INIHandler::INIHandler()
{

}

/*
 * Load datas form ini file.
*/
bool INIHandler::loadIni(QString file_path)
{
    QFile ini(file_path);

    if(ini.exists() == false)
    {
        return false;
    }

    ini.open(QIODevice::ReadOnly);

    QString tmp = ini.readLine();

    while(!tmp.isNull())
    {
        tmp = tmp.trimmed();
        int first = tmp.indexOf("=");
        if(first >= 1)
        {
            QString key = tmp.left(first);
            QString value = tmp.mid(first + 1);

            o_map.insert(key, value);
        }
        tmp = ini.readLine();
    }

    ini.close();
    return true;
}

/*
 * Check and get QMap value via key
 */
QString INIHandler::checkAndGetValue(QString key, int *ok)
{
    if(!o_map.contains(key))
    {
        if(ok != nullptr)
        {
            *ok = GET_NOKEY;
            return "";
        }
    }
    if(ok != nullptr)
        *ok = GET_SUCCESS;

    return o_map.value(key);
}

/*
 * Check and get QMap value via key then parse to int
 */
int INIHandler::checkAndGetValueInt(QString key, int *ok)
{
    QString t = checkAndGetValue(key, ok);
    if(ok != nullptr && *ok != GET_SUCCESS)
        return 0;
    bool ok2;
    int re = t.toInt(&ok2);
    if(!ok2)
    {
        if(ok != nullptr)
            *ok = GET_NOTINT;
        return 0;
    }
    if(ok != nullptr)
        *ok = GET_SUCCESS;
    return re;
}
