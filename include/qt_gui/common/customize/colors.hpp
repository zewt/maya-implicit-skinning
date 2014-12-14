/*
 Implicit skinning
 Copyright (C) 2013 Rodolphe Vaillant, Loic Barthe, Florian Cannezin,
 Gael Guennebaud, Marie Paule Cani, Damien Rohmer, Brian Wyvill,
 Olivier Gourmel

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License 3 as published by
 the Free Software Foundation.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>
 */
#ifndef COLORS_DIALOG_HPP__
#define COLORS_DIALOG_HPP__

#include <map>
#include <string>

#include "ui_colors.h"
#include "maths/color.hpp"

/** @class Colors_dialog
  @brief A dialog box in order to change various colors used in the engine

  The dialog box displays a clickable textual list of colors that can be changed.
  Colors are stored in the Color_ctrl class. You can find a global instance of
  this class in the Cuda_ctrl namespace in file cuda_ctrl.hpp.

  One can add new colors by adding an item to the list of the QListWidget.
  The association between an item and its color stored in Color_ctrl must
  be explicitly defined with the std::map attribute _map_item_enum.
  The std::map is filled inside the constructor. The line :
  @code
  _map_item_enum["the new item text"]  = Color_ctrl::A_NEW_ENUM_FIELD_IN_COLOR_CTRL;
  @endcode
  is to be added to the constructor

  @see Color_ctrl Cuda_ctrl::_color
*/
class Colors_dialog : public QDialog, public Ui::ColorDiag
{
    Q_OBJECT

public:

    Colors_dialog(QWidget *parent = 0);
    ~Colors_dialog();

private:

    /// return the color associated to the item and change its value to cl
    /// if update_cl is true
    Color get_color_and_set(const QListWidgetItem* item,
                            bool update_cl = false,
                            Color cl = Color(0.f, 0.f, 0.f, 0.f));

    std::map<std::string, int> _map_item_enum;

public slots:

    // ...

private slots:
    void validBox();
    void color_preview_clicked();
    void on_color_elements_list_itemClicked(QListWidgetItem* item);
};

#endif
