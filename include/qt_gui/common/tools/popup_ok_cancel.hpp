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
#include <QDialog>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>

class Diag_ok_cancel : public QDialog {
    Q_OBJECT

public:
    Diag_ok_cancel( const QString& name, const QString& txt, QWidget *parent) :
        QDialog( parent )
    {
        this->setWindowTitle(name);

        // Setup layouts
        QVBoxLayout* vlayout = new QVBoxLayout(this);
        QWidget* widget      = new QWidget(this);
        QHBoxLayout* hlayout = new QHBoxLayout(widget);

        // Add label
        QLabel* lbl;
        lbl = new QLabel(txt, this);
        vlayout->addWidget(lbl);
        vlayout->addWidget(widget);

        // Add buttons
        QPushButton *ok, *cancel;
        ok = new QPushButton( "OK", this );
        ok->setGeometry( 10,10, 100,30 );
        connect( ok, SIGNAL(clicked()), SLOT(accept()) );
        cancel = new QPushButton( "Cancel", this );
        cancel->setGeometry( 10,60, 100,30 );
        connect( cancel, SIGNAL(clicked()), SLOT(reject()) );

        // Add widgets to layout
        hlayout->addWidget(ok);
        hlayout->addWidget(cancel);
    }
};
