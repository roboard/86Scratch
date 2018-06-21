/* 
  animefilehandle.cpp - DM&P 86Scratch
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

#include "animefilehandle.h"

#include <QDir>

AnimeFileHandle::AnimeFileHandle(QString path, bool circle)
{
    anime_files.clear();

    QDir images(path);
    images.setFilter(QDir::Files | QDir::NoDotAndDotDot);
    anime_files = images.entryList();
    current_index = 0;

    parent_dir = path;
    is_circle = circle;
}

QString AnimeFileHandle::getNextImagePath()
{
    if(anime_files.size() == 0)
        return "";
    else if(current_index == anime_files.size())
        current_index--;

    QString t = parent_dir + "/" + anime_files.at(current_index);
    current_index++;
    if(current_index == anime_files.size() && is_circle)
    {
        current_index = 0;
    }

    return t;
}

void AnimeFileHandle::resetIndex()
{
    current_index = 0;
}

bool AnimeFileHandle::isEnd()
{
    return current_index == anime_files.size() && !is_circle;
}

