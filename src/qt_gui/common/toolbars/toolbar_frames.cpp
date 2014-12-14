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
#include "common/toolbars/toolbar_frames.hpp"

#include <QLabel>
#include <QString>
#include <QFileDialog>
#include <limits>

#include "std_utils.hpp"
#include "globals.hpp"

// Export cuda_ctrl.hpp --------------------------------------------------------
#include "skeleton_ctrl.hpp"
namespace Cuda_ctrl {
extern Skeleton_ctrl _skeleton;
}
// End Export cuda_ctrl.hpp ----------------------------------------------------

Toolbar_frames::~Toolbar_frames()
{
    for(unsigned i = 0; i < _anim_list.size(); i++)
        delete _anim_list[i];
}

// -----------------------------------------------------------------------------

Toolbar_frames::Toolbar_frames(QWidget* parent) :
    QToolBar(parent),
    _ico_rec_off("./resource/icons/rec.png"),
    _ico_rec_on("./resource/icons/rec_on.png"),
    _ico_play("./resource/icons/play.png"),
    _ico_pause("./resource/icons/pause.png"),
    _play(false),
    _frame(0),
    _curr_anim(-1)
{

    //Setup layout
    QLayout* layout = this->layout();
    layout->setSpacing(6);
    layout->setObjectName(QString::fromUtf8("_hLayout"));
    //_hLayout->setContentsMargins(-1, 0, -1, 0);
    // -----------------

    // Adding toolButtons for timeline navigation
    _buttons = new Widget_frame(this);
    QObject::connect(_buttons->toolB_play, SIGNAL(pressed()), this, SLOT(play()));
    QObject::connect(_buttons->toolB_stop, SIGNAL(pressed()), this, SLOT(stop()));
    QObject::connect(_buttons->toolB_prev, SIGNAL(pressed()), this, SLOT(prev()));
    QObject::connect(_buttons->toolB_next, SIGNAL(pressed()), this, SLOT(next()));
    this->addWidget( _buttons );

    // Setup record button
    // TODO: _rec_button should be integrated in Widget_frame
    _rec_button = new QToolButton(this);
    _rec_button->setMinimumSize(36, 36);
    _rec_button->setObjectName("rec_button");
    _rec_button->setIcon(_ico_rec_off);
    _rec_button->setIconSize(QSize(32, 32));
    _rec_button->setToolTip("Record animation (Mesh and/or screenshots)");
    _rec_state = false;

    QObject::connect(_rec_button, SIGNAL(released()), this, SLOT(rec_button_released()));
    _buttons->horizontalLayout->addWidget(_rec_button);
    // -----------------

    // Setup screenshot button
    _img_button = new QToolButton(this);
    _img_button->setMinimumSize(36, 36);
    _img_button->setObjectName("img_button");
    _img_button->setIcon(QIcon("./resource/icons/img_thumb.png"));
    _img_button->setIconSize(QSize(32, 32));
    _img_button->setToolTip("Enable screenshots");
    _img_button->setAutoRaise(true);
    _img_button->setCheckable(true);
    _shot_state = false;

    QObject::connect(_img_button, SIGNAL(toggled(bool)), this, SLOT(img_button_toogle(bool)));
    _buttons->horizontalLayout->addWidget(_img_button);
    this->addSeparator();
    // -----------------

    // Setup spinBox to display frame number
    _dSpinB_fps = new QDoubleSpinBox(this);
    _dSpinB_fps->setRange(0., std::numeric_limits<double>::max());
    _dSpinB_fps->setSingleStep(5.);
    _dSpinB_fps->setValue( 0. );
    _dSpinB_fps->setMaximumSize(16+16*3, 32);
    QObject::connect(_dSpinB_fps, SIGNAL(valueChanged(double)), this, SLOT(fps_changed(double)));
    this->addWidget(_dSpinB_fps);
    // -----------------

    // Setup spinBox to display frame number
    _spinB_frames = new QSpinBox(this);
    _spinB_frames->setRange(0, std::numeric_limits<int>::max());
    _spinB_frames->setDisabled(true);
    _spinB_frames->setMaximumSize(16+16*3, 32);
    this->addWidget(_spinB_frames);
    // -----------------

    // Setup timeline slider
    _slider = new QSlider(this);
    _slider->setObjectName(QString::fromUtf8("_slider"));
    //_slider->setEnabled(false);
    _slider->setMaximum(100);
    _slider->setMinimum(0);
    _slider->setValue(0);
    _slider->setOrientation(Qt::Horizontal);
    _slider->setInvertedAppearance(false);
    _slider->setInvertedControls(false);
    _slider->setTickPosition(QSlider::TicksBelow);
    QObject::connect(_slider, SIGNAL(valueChanged(int)), this, SLOT(set_frame_number(int)));
    this->addWidget(_slider);
    // -----------------

    // Setup timer to play animation
    connect(&_timer_frame, SIGNAL(timeout()), this, SLOT(next()));
    // -----------------

    // Setup combo box for the animation list
    _anim_box = new QComboBox(this);
    _anim_names.push_back( "none" );
    _anim_box->addItem( QString("none") );
    QObject::connect(_anim_box, SIGNAL(currentIndexChanged(int)),
                     this     , SLOT(anim_box_index_changed(int)));
    this->addWidget(_anim_box);
    // -----------------

    // Setup label
    QLabel* lbl = new QLabel("Export path", this);
    this->addWidget(lbl);
    // -----------------


    // Setup line edit to display point cache animation exporter path
    _line_edit = new QLineEdit(QString(g_write_dir.c_str()),this);
    _line_edit->setMaximumSize(16+16*6, 32);
    this->addWidget(_line_edit);
    // -----------------


    // Setup path button to browse and set the filepath to export anim
    _path_button = new QPushButton("...", this);
    _path_button->setMaximumSize(16+16*2, 32);
    QObject::connect(_path_button, SIGNAL(released()),
                     this       , SLOT(path_button_released()));
    this->addWidget(_path_button);
    // -----------------
}

// -----------------------------------------------------------------------------

/// Compute the minimal int suffix 'depth' needed for 'str' to be unique in the
/// vector 'vec'
/// @return a string which does not match any element in 'vec'
static std::string gen_unique_str(const std::vector<std::string>& vec,
                                  std::string str,
                                  int depth)
{
    std::string n_str = str + " (" + Std_utils::to_string(depth) + ")";
    if( !Std_utils::exists(vec, n_str) )
        return n_str;
    else
        return gen_unique_str(vec, str, depth+1);
}

// -----------------------------------------------------------------------------

void Toolbar_frames::set_anim_list(
        const std::vector<Loader::Base_anim_eval*>& list)
{
    for(unsigned i = 0; i < _anim_list.size(); i++)
        delete _anim_list[i];

    _curr_anim = -1;
    _anim_box->clear();
    _anim_names.clear();
    _anim_list. clear();
    add_anims( list );
}

// -----------------------------------------------------------------------------

void Toolbar_frames::add_anims(
        const std::vector<Loader::Base_anim_eval*>& list)
{
    const int start = _anim_names.size();
    // Always append non to the beginning
    if(start == 0) _anim_names.push_back( "none" );

    _anim_names.reserve( _anim_names.size() + list.size() );
    _anim_list. reserve( _anim_list.size()  + list.size() );

    for(unsigned i = 0; i < list.size(); i++)
    {
        std::string name = list[i]->_name;
        if( Std_utils::exists(_anim_names, name) )
            name = gen_unique_str(_anim_names, name, 2);

        _anim_list. push_back( list[i] );
        _anim_list[_anim_list.size()-1]->_name = name;
        _anim_names.push_back( name    );
    }

    for(unsigned i = start; i < _anim_names.size(); i ++)
        _anim_box->addItem( QString( _anim_names[i].c_str() ) );
}

// -----------------------------------------------------------------------------

void Toolbar_frames::update_scene()
{
    if(_curr_anim >= 0){
        Cuda_ctrl::_skeleton.set_pose(_anim_list[_curr_anim], _frame);
        emit update_gl();
    }
}

// -----------------------------------------------------------------------------

enum Anim_t {MDD, PC2};
void Toolbar_frames::export_anim(Anim_t t)
{
    std::string filepath = _line_edit->text().toStdString()+"/anim_export.mdd";
    if(g_anim_cache != 0)
        g_anim_cache->export_mdd(filepath);
}

// -----------------------------------------------------------------------------

void Toolbar_frames::rec_button_released()
{
    if(_rec_state)
    {
        // Disable recording
        _rec_button->setIcon( _ico_rec_off );
        _rec_state = false;
        g_shooting_state = false; // Disable screen shot
        export_anim(MDD);
        g_save_anim = false;
        //_spinB_frames->setValue(0);
    }
    else
    {
        // Enable recording
        _rec_button->setIcon(_ico_rec_on);
        _rec_state = true;
        g_shooting_state = _shot_state;
        g_save_anim = true;
    }
}

// -----------------------------------------------------------------------------

void Toolbar_frames::img_button_toogle(bool t){ _shot_state = t; }

// -----------------------------------------------------------------------------

void Toolbar_frames::path_button_released()
{
    QString path = QFileDialog::getExistingDirectory(this,
                                                     tr("Export directory"),
                                                     _line_edit->text());
    _line_edit->setText(path);
}

// -----------------------------------------------------------------------------

void Toolbar_frames::anim_box_index_changed(int idx)
{
    std::string name = _anim_box->itemText( idx ).toStdString();
    unsigned i;
    for(i = 0; i < _anim_list.size(); i++)
        if( _anim_list[i]->_name.compare(name) == 0 )
            break;

    if(i < _anim_list.size()){
        _curr_anim = i;
        _dSpinB_fps->setValue( _anim_list[_curr_anim]->frame_rate() );
        _slider->setMaximum( _anim_list[i]->nb_frames()-1 );
    } else {
        _slider->setMaximum( 100 );
        _curr_anim = -1;
    }

    set_frame_number(0);
}

// -----------------------------------------------------------------------------

void Toolbar_frames::set_frame_number(int n)
{
    _spinB_frames->setValue(n);
    _slider->setValue(n);
    _frame = n;
    update_scene();
}

// -----------------------------------------------------------------------------

void Toolbar_frames::prev()
{
    _frame = (_frame-1) >= 0 ? (_frame-1) : 0;
    set_frame_number(_frame);
}

// -----------------------------------------------------------------------------

void Toolbar_frames::next()
{
    if( _curr_anim >= 0 )
    {
        _frame = (_frame+1) % _anim_list[_curr_anim]->nb_frames();
        if( _frame == 0 ){
            _timer_frame.stop();
            set_pause();
        }
    }
    set_frame_number(_frame);
}

// -----------------------------------------------------------------------------

void Toolbar_frames::fps_changed(double fps)
{
    if( _play ){
        int ifps = fps > 0. ? (1. / fps) * 1e3 : 0;
        _timer_frame.start( ifps );
    }
}

// -----------------------------------------------------------------------------

void Toolbar_frames::play()
{
    if( _curr_anim < 0) return;

    if( _play ) {
        _timer_frame.stop();
        set_pause();
    } else {
        double fps = _dSpinB_fps->value();
        int ifps = fps > 0. ? (1. / fps) * 1e3 : 0;
        _timer_frame.start( ifps );

        set_play();
    }
}

// -----------------------------------------------------------------------------

void Toolbar_frames::set_pause()
{
    _buttons->toolB_play->setIcon( _ico_play );
    _play = false;
}

// -----------------------------------------------------------------------------

void Toolbar_frames::set_play()
{
    _buttons->toolB_play->setIcon( _ico_pause );
    _play = true;
}

// -----------------------------------------------------------------------------

void Toolbar_frames::stop()
{
    _timer_frame.stop();
    set_pause();
    set_frame_number( 0 );
}

// -----------------------------------------------------------------------------
