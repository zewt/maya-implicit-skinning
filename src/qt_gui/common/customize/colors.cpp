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
#include "common/customize/colors.hpp"

#include <QColorDialog>
#include <QMessageBox>
#include "cuda_ctrl.hpp"

// -----------------------------------------------------------------------------

static QColor ColorToQColor( const Color& cl)
{
    return QColor((int)(cl.r*255.f), (int)(cl.g*255.f), (int)(cl.b*255.f));
}

// -----------------------------------------------------------------------------

static Color qColorToColor(const QColor& cl)
{
    Color nCl = Color((float)cl.redF(), (float)cl.greenF(), (float)cl.blueF(), 1.f);
    return nCl;
}

// -----------------------------------------------------------------------------

Color Colors_dialog::get_color_and_set(const QListWidgetItem* item,
                                       bool update_cl,
                                       Color cl)
{
    std::map<std::string,int>::iterator it;
    it = _map_item_enum.find( item->text().toStdString() );

    if(it != _map_item_enum.end())
    {
        int field = it->second;

        Color c = Cuda_ctrl::_color.get(field);
        if(update_cl) Cuda_ctrl::_color.set(field, cl);
        return c;
    }

    return Color();
}

// -----------------------------------------------------------------------------

Colors_dialog::Colors_dialog(QWidget *parent) : QDialog(parent)
{
    setupUi(this);

    color_elements_list->sortItems();

    //////////////////////////////////////////
    // Associates item text with enum field //
    //////////////////////////////////////////

    // TODO: store this map in color_ctrl. assert in color_ctrl if map size != from enum size
    _map_item_enum["viewport background"] = Color_ctrl::BACKGROUND;
    _map_item_enum["hermite rbf points"] = Color_ctrl::HRBF_POINTS;
    _map_item_enum["hermite rbf selected points"] = Color_ctrl::HRBF_SELECTED_POINTS;
    _map_item_enum["mesh points on sides and not manifold"] = Color_ctrl::MESH_DEFECTS;
    _map_item_enum["mesh points and wires"] = Color_ctrl::MESH_POINTS;
    _map_item_enum["mesh selected points"]  = Color_ctrl::MESH_SELECTED_POINTS;
    _map_item_enum["mesh outline"]  = Color_ctrl::MESH_OUTLINE;
    _map_item_enum["viewport messages"]  = Color_ctrl::VIEWPORTS_MSGE;
    _map_item_enum["intern potential"]  = Color_ctrl::POTENTIAL_IN;
    _map_item_enum["extern potential"]  = Color_ctrl::POTENTIAL_OUT;
    _map_item_enum["negative potential"]  = Color_ctrl::POTENTIAL_NEG;
    _map_item_enum["huge potential"]  = Color_ctrl::POTENTIAL_POS;

    // Fill the widget list given the map
    std::map<std::string, int>::iterator it;
    for(it = _map_item_enum.begin(); it != _map_item_enum.end(); ++it)
        color_elements_list->addItem( QString(it->first.c_str()) );

    ///////////////////////////////////////////////////////
    // Set color preview with the first item in the list //
    ///////////////////////////////////////////////////////
    if(color_elements_list->count() > 0)
    {
        QListWidgetItem* it = color_elements_list->item(0);
        color_elements_list->setCurrentItem(it);
        Color cl = get_color_and_set(it, false);
        color_preview->set_background( ColorToQColor(cl) );
    }

    ///////////////////
    // Connect slots //
    ///////////////////

    // Catch ok_button pressed signal to close the window
    QObject::connect(this->ok_button, SIGNAL(pressed()), this, SLOT(validBox()));
    // Connect the color preview pressed event to show a QColorDialog
    QObject::connect(this->color_preview, SIGNAL(leftclick_on()), this, SLOT(color_preview_clicked()));
}

// -----------------------------------------------------------------------------

Colors_dialog::~Colors_dialog()
{
    // ...
}

// -----------------------------------------------------------------------------

void Colors_dialog::validBox()
{
    // Do some stuff with values...
    this->close();
}

// -----------------------------------------------------------------------------

void Colors_dialog::color_preview_clicked()
{
    QListWidgetItem* it = color_elements_list->currentItem();
    if( it != 0)
    {
        Color cl = get_color_and_set(it, false);
        QColor c = QColorDialog::getColor( ColorToQColor(cl), this, "Choose color" );
        color_preview->set_background(c);
        get_color_and_set(it, true, qColorToColor(c));
    }
    else
    {
        QMessageBox::information(this,"Error" ,"Please select a color to change.");
    }
}

// -----------------------------------------------------------------------------

void Colors_dialog::on_color_elements_list_itemClicked(QListWidgetItem* item)
{
    Color cl = get_color_and_set(item, false);
    color_preview->set_background( ColorToQColor(cl) );
}

// -----------------------------------------------------------------------------
