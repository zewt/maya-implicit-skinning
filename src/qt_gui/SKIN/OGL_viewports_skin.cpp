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
#include "SKIN/OGL_viewports_skin.hpp"

#include <QSplitter>

#include "cuda_ctrl.hpp"
#include "macros.hpp"

// Export from global.hpp //////////
#include "glshoot.hpp"
extern GlShoot* g_oglss;
extern bool g_shooting_state;
extern std::string g_write_dir;
///////////////////////////////////

// -----------------------------------------------------------------------------

void OGL_widget_skin_hidden::initializeGL(){
    OGL_widget_skin::init_glew();
    //QGLWidget::initializeGL();
    init_opengl();
}

// -----------------------------------------------------------------------------

OGL_viewports_skin::OGL_viewports_skin(QWidget* w, Main_window_skin* m) :
    QFrame(w),
    _skel_mode(false),
    _io_type(EOGL_widget::DISABLE),
    _current_viewport(0),
    _main_layout(0),
    _main_window(m),
    _frame_count(0)
{

    // Initialize opengl shared context
    _hidden = new OGL_widget_skin_hidden(this);
    // Needed to initialize the opengl shared context
    _hidden->updateGL();
    _hidden->makeCurrent();

    OGL_widget_skin::init_glew();

    _hidden->setGeometry(0,0,0,0);
    _hidden->hide();
    assert( _hidden->isValid() );


    // Initialize cuda context
    std::vector<Blending_env::Op_t> op;
//    op.push_back( Blending_env::B_TCH );
    op.push_back( Blending_env::B_D  );
    op.push_back( Blending_env::U_OH );
    op.push_back( Blending_env::C_D  );

//    for (int i=(int)Blending_env::BINARY_3D_OPERATOR_BEGIN+1; i<(int)Blending_env::BINARY_3D_OPERATOR_END; ++i)
//        op.push_back( Blending_env::Op_t(i) );

    Cuda_ctrl::cuda_start( op );
    Cuda_ctrl::init_opengl_cuda();
    g_oglss = new GlShoot(10, 10, g_write_dir, "shot");

    set_viewports_layout(SINGLE);
    QObject::connect(this, SIGNAL(active_viewport_changed(int)),
                     m   , SLOT(active_viewport(int)) );

    QObject::connect(this        , SIGNAL(update_status(QString)),
                     m->statusbar, SLOT(showMessage(QString)) );
}

// -----------------------------------------------------------------------------

OGL_viewports_skin::~OGL_viewports_skin(){
    delete g_oglss;
}

// -----------------------------------------------------------------------------

/////////////////DEBUG
/// The current skeleton
#if 0
// HARD CODED Anim
extern Skeleton* g_skel;

Kinematic* kinec(){ return g_skel->_kinec; }

void rotate(const Vec3_cu& axis, float angle, int joint)
{
    /*-----------------
      Compute rotation
    ------------------*/
    int pid = g_skel->parent(joint) > -1  ? g_skel->parent(joint) : joint;

    Transfo pframe     = g_skel->joint_anim_frame( pid );
    Transfo pframe_inv = pframe.fast_invert();

    // Axis in world coordinates
    Vec3_cu world_axis = axis;

    // Joint origin in its parent joint coordinate system
    Vec3_cu curr_joint_org = g_skel->joint_anim_frame(joint).get_translation();
    Point_cu org = pframe_inv * curr_joint_org.to_point();

    // Rotation of the joint in its parent joint coordinate system
    Transfo tr = Transfo::rotate(org.to_vector(), pframe_inv * world_axis, angle);

    Transfo curr_joint_lcl = kinec()->get_user_lcl_parent( joint );
    // Concatenate last user defined transformation
    Transfo usr = tr * curr_joint_lcl;

    // Update the skeleton position
    kinec()->set_user_lcl_parent( joint, usr );
}
#endif
/////////////////DEBUG



void OGL_viewports_skin::updateGL()
{
    using namespace Cuda_ctrl;
    _viewports[0]->makeCurrent();

    // FPS Counting _______________
    static int fpsCount = -1;
    static int fpsLimit = 11;
    static double fps_min = 100000.;
    static double fps_max = 0.;
    static double fps_avg = 0.;
    static int nb_frames_avg = 0;
    static unsigned elapsed = 0.;
    if(fpsCount == -1) fpsCount = 0;
    QTime  frame_timer;
    frame_timer.start();
    if(fpsCount == 0)
        _fps_timer.start();
    // _____________________________


    if(_skel_mode)
    {
        if(_anim_mesh != 0)
        {
            #if 0
            // HARD CODED Anim
            const float step = 0.08f;
            static int sign = 1;
            static float acc  = 0;

            acc = acc + sign*step;

            if(acc > ((2.2f*M_PI) / 3.f))
                sign *= -1;

            if(acc <  0.f){
                sign *= -1;
                g_skel->reset();
            }

            rotate(Vec3_cu::unit_y(), step*sign, 11);
            #endif

            // Transform HRBF samples for display and selection
            _anim_mesh->transform_samples( /*_skeleton.get_selection_set() */);
            // Animate the mesh :
            _anim_mesh->deform_mesh();
        }
    }

    for(unsigned i = 0; i < _viewports.size(); i++)
    {
        OGL_widget_skin* v = _viewports[i];
        v->updateGL();
        // Screenshot of the active viewport
        if(g_shooting_state && v == active_viewport())
        {
            g_oglss->set_img_size(v->width(), v->height());
            g_oglss->shoot();
        }
    }

    // FPS Counting ________________
    double this_frame = frame_timer.elapsed();
    fpsCount++;
    if (fpsCount >= fpsLimit)
    {
        // We don't count the last frame.
        fpsCount -= 1;
        elapsed = _fps_timer.elapsed() - this_frame;

        double ifps =  (double)fpsCount / ((double)elapsed / 1000.);
        if(ifps < fps_min) fps_min = ifps;
        if(ifps > fps_max) fps_max = ifps;
        fps_avg += ifps;
        nb_frames_avg++;
        QString msg = QString::number(ifps) +" fps ";
        msg = msg+"min: "+QString::number(fps_min)+" ";
        msg = msg+"max: "+QString::number(fps_max)+" ";
        msg = msg+"avg: "+QString::number(fps_avg/nb_frames_avg)+" ";
        msg = msg+"Implicit Visualizer:"+QString::number(ifps)+"fps, ";

        if(_current_viewport != 0)
        {
            double w = _current_viewport->camera()->width();
            double h = _current_viewport->camera()->height();
            double frame_to_Mrays = w*h*MULTISAMPX*MULTISAMPY*(1e-6);
            msg = msg+QString::number(ifps*frame_to_Mrays)+" Mray/s";
            msg = msg+" res:"+QString::number(width())+"x"+QString::number(height());
        }

        emit update_status(msg);

        if(nb_frames_avg >= fpsLimit*2){
            nb_frames_avg = 0;
            fps_avg = 0.;
            fps_min = 1000.;
            fps_max = 0.;
        }
        fpsCount = 0;
        elapsed = 0;
    }
    // _____________________________
}

// -----------------------------------------------------------------------------

void OGL_viewports_skin::enterEvent( QEvent* e){
    if(_current_viewport != 0)
        _current_viewport->setFocus();
    e->ignore();
}

// -----------------------------------------------------------------------------

void OGL_viewports_skin::set_viewports_layout(Layout_e setting)
{
    erase_viewports();

    switch(setting)
    {
    case SINGLE:  _main_layout = gen_single (); break;
    case VDOUBLE: _main_layout = gen_vdouble(); break;
    case HDOUBLE: _main_layout = gen_hdouble(); break;
    case FOUR:    _main_layout = gen_four   (); break;
    }

    this->setLayout(_main_layout);
    first_viewport_as_active();
    set_io(_io_type);
}

// -----------------------------------------------------------------------------

QLayout* OGL_viewports_skin::gen_single()
{
    QVBoxLayout* vlayout = new QVBoxLayout(this);
    vlayout->setSpacing(0);
    vlayout->setContentsMargins(0, 0, 0, 0);

    {
        Viewport_frame_skin* frame = new_viewport_frame(this, 0);
        _viewports_frame.push_back(frame);
        QVBoxLayout* layout = new QVBoxLayout(frame);
        layout->setSpacing(0);
        layout->setContentsMargins(0, 0, 0, 0);
        frame->setLayout(layout);

        OGL_widget_skin* ogl = new_viewport(frame);
        _viewports.push_back(ogl);
        layout->addWidget(ogl);

        vlayout->addWidget(frame);
    }

    return vlayout;
}

// -----------------------------------------------------------------------------

QLayout* OGL_viewports_skin::gen_vdouble()
{

    QVBoxLayout* vlayout = new QVBoxLayout(this);
    vlayout->setSpacing(0);
    vlayout->setContentsMargins(0, 0, 0, 0);

    QSplitter* splitter = new QSplitter(this);
    splitter->setOrientation(Qt::Horizontal);
    splitter->setContentsMargins(0, 0, 0, 0);
    splitter->setHandleWidth(3);
    vlayout->addWidget(splitter);

    for(int i = 0; i < 2; i++){
        Viewport_frame_skin* frame = new_viewport_frame(splitter, i);
        _viewports_frame.push_back(frame);
        QVBoxLayout* layout = new QVBoxLayout(frame);
        layout->setSpacing(0);
        layout->setContentsMargins(0, 0, 0, 0);
        frame->setLayout(layout);

        OGL_widget_skin* ogl = new_viewport(frame);
        _viewports.push_back(ogl);
        layout->addWidget(ogl);

        splitter->addWidget(frame);
    }

    return vlayout;
}

// -----------------------------------------------------------------------------

QLayout* OGL_viewports_skin::gen_hdouble()
{

    QVBoxLayout* vlayout = new QVBoxLayout(this);
    vlayout->setSpacing(0);
    vlayout->setContentsMargins(0, 0, 0, 0);

    QSplitter* splitter = new QSplitter(this);
    splitter->setOrientation(Qt::Vertical);
    splitter->setContentsMargins(0, 0, 0, 0);
    splitter->setHandleWidth(3);
    vlayout->addWidget(splitter);

    for(int i = 0; i < 2; i++){
        Viewport_frame_skin* frame = new_viewport_frame(splitter, i);
        _viewports_frame.push_back(frame);
        QVBoxLayout* layout = new QVBoxLayout(frame);
        layout->setSpacing(0);
        layout->setContentsMargins(0, 0, 0, 0);
        frame->setLayout(layout);

        OGL_widget_skin* ogl = new_viewport(frame);
        _viewports.push_back(ogl);
        layout->addWidget(ogl);

        splitter->addWidget(frame);
    }
    return vlayout;
}

// -----------------------------------------------------------------------------

QLayout* OGL_viewports_skin::gen_four()
{

    QVBoxLayout* vlayout = new QVBoxLayout(this);
    vlayout->setSpacing(0);
    vlayout->setContentsMargins(0, 0, 0, 0);

    QSplitter* vsplitter = new QSplitter(this);
    vsplitter->setOrientation(Qt::Vertical);
    vsplitter->setContentsMargins(0, 0, 0, 0);
    vsplitter->setHandleWidth(3);
    vlayout->addWidget(vsplitter);

    int acc = 0;
    for(int i = 0; i < 2; i++)
    {
        QSplitter* hsplitter = new QSplitter(this);
        hsplitter->setOrientation(Qt::Horizontal);
        hsplitter->setContentsMargins(0, 0, 0, 0);
        hsplitter->setHandleWidth(3);
        vsplitter->addWidget(hsplitter);

        for(int j = 0; j < 2; j++)
        {
            Viewport_frame_skin* frame = new_viewport_frame(hsplitter, acc);
            acc++;
            _viewports_frame.push_back(frame);
            QVBoxLayout* layout = new QVBoxLayout(frame);
            layout->setSpacing(0);
            layout->setContentsMargins(0, 0, 0, 0);
            frame->setLayout(layout);

            OGL_widget_skin* ogl = new_viewport(frame);
            _viewports.push_back(ogl);
            layout->addWidget(ogl);

            hsplitter->addWidget(frame);
        }
    }

    return vlayout;
}

// -----------------------------------------------------------------------------

void OGL_viewports_skin::erase_viewports()
{
    for(unsigned i = 0; i < _viewports.size(); i++)
    {
        _viewports[i]->close();
        delete _viewports[i];
    }

    // We don't need to delete the frames because qt will do it when deleting
    // the main layout
    _viewports_frame.clear();
    _viewports.clear();
    delete _main_layout;
    _main_layout = 0;
}

// -----------------------------------------------------------------------------

OGL_widget_skin* OGL_viewports_skin::new_viewport(Viewport_frame_skin* ogl_frame)
{
    OGL_widget_skin* ogl = new OGL_widget_skin(ogl_frame, _hidden, _main_window);
    QObject::connect(ogl, SIGNAL(drawing()), this, SLOT(incr_frame_count()));
    QObject::connect(ogl, SIGNAL( clicked() ), ogl_frame, SLOT( activate() ));
    ogl->setMinimumSize(4, 4);
    // initialize openGL and paint widget :
    ogl->updateGL();
    return ogl;
}

// -----------------------------------------------------------------------------

Viewport_frame_skin* OGL_viewports_skin::new_viewport_frame(QWidget* parent, int id)
{
    Viewport_frame_skin* frame = new Viewport_frame_skin(parent, id);
    QObject::connect(frame, SIGNAL( active(int) ),
                     this , SLOT  ( active_viewport_slot(int)) );

    return frame;
}

// -----------------------------------------------------------------------------

void OGL_viewports_skin::set_frame_border_color(Viewport_frame_skin* f, int r, int g, int b)
{
    QString str = "color: rgb("+
            QString::number(r)+", "+
            QString::number(g)+", "+
            QString::number(b)+");";

    f->setStyleSheet( str );
}

// -----------------------------------------------------------------------------

Vec_viewports& OGL_viewports_skin::get_viewports()
{
    return _viewports;
}

// -----------------------------------------------------------------------------

void OGL_viewports_skin::set_io(EOGL_widget::IO_t io_type)
{
    _io_type = io_type;
    bool state;
    switch(io_type)
    {
    case EOGL_widget::RBF:       state = true;  break;
    case EOGL_widget::DISABLE:   state = false; break;
    case EOGL_widget::GRAPH:     state = false; break;
    case EOGL_widget::SKELETON:  state = true;  break;
    case EOGL_widget::MESH_EDIT: state = true;  break;
    case EOGL_widget::BLOB:      state = false;  break;
    default:                    state = true;  break;
    }

    _skel_mode = _skel_mode || state;

    for(unsigned i = 0; i < _viewports.size(); i++){
        _viewports[i]->set_io(io_type);
        _viewports[i]->set_draw_skeleton(state);
    }
}

// -----------------------------------------------------------------------------

void OGL_viewports_skin::set_pivot_mode(EOGL_widget::Pivot_t m)
{
    for(unsigned i = 0; i < _viewports.size(); i++)
        _viewports[i]->set_pivot_mode(m);
}

// -----------------------------------------------------------------------------

void OGL_viewports_skin::incr_frame_count()
{
    _frame_count++;
    emit frame_count_changed(_frame_count);
}

// -----------------------------------------------------------------------------

void OGL_viewports_skin::active_viewport_slot(int id)
{
    for(unsigned i = 0; i < _viewports_frame.size(); i++)
        set_frame_border_color(_viewports_frame[i], 0, 0, 0);

    set_frame_border_color(_viewports_frame[id], 255, 0, 0);
    _current_viewport = _viewports[id];

    _current_viewport->setFocus();
    _current_viewport->makeCurrent();
    emit active_viewport_changed(id);
}

// -----------------------------------------------------------------------------

void OGL_viewports_skin::first_viewport_as_active()
{
    assert(_viewports.size() > 0);
    _current_viewport = _viewports[0];
    set_frame_border_color(_viewports_frame[0], 255, 0, 0);
    _current_viewport->setFocus();
    _current_viewport->makeCurrent();
}

// -----------------------------------------------------------------------------
