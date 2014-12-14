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
#include "SKIN/main_window_skin.hpp"

#include "cuda_ctrl.hpp"
#include "point_cache_export.hpp"
#include "SKIN/OGL_viewports_skin.hpp"
#include "globals.hpp"

// From cuda_globals.hpp -------
#include "skeleton.hpp"
extern Skeleton* g_skel;
// -------

// -----------------------------------------------------------------------------

void Main_window_skin::keyPressEvent( QKeyEvent* event )
{
    using namespace Cuda_ctrl;

    OGL_widget_skin* wgl = _viewports->active_viewport();

    // Check special keys
    switch( event->key() ){
    case Qt::Key_F1:
        _debug._smooth_mesh = !_debug._smooth_mesh;
        checkB_enable_smoothing->setChecked(!checkB_enable_smoothing->isChecked());
        break;
    case Qt::Key_F4:
        wgl->set_rest_pose( !wgl->rest_pose() );
        break;
    case Qt::Key_Backspace:
        g_skel->_kinec->reset();
        break;
    }

    // Check standard characters :
    QString t = event->text();
    QChar c = t[0];
    using namespace Constants;
    switch (c.toLatin1()) {
    case 'o':
    {
        if( _anim_mesh != 0)
        {
            _anim_mesh->switch_implicit_skinning();
            bool state = !implicit_skinning_checkBox->isChecked();
            implicit_skinning_checkBox->      setChecked( state );
            toolBar->_wgt_fit->toolB_fitting->setChecked( state );
        }
    }break;
    case 'r':
    {
        wgl->set_raytracing( !wgl->raytrace() );
        settings_raytracing->enable_raytracing->setChecked( wgl->raytrace() );
    }break;
    case 'T':
    {
        wgl->set_phong( !wgl->phong_rendering() );
    }break;
        // escape key :
    case 27:
    {
        exit(0); // free memory because cuda_start() sets atexit() to call a memory cleaner function
    }break;
    case 't': wgl->set_draw_mesh( !wgl->draw_mesh() ); break;
    case 'g': _display._grid = !_display._grid; break;
    }

    _viewports->updateGL();
}
