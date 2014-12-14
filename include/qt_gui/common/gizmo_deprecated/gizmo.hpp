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
#ifndef GIZMO_HPP__
#define GIZMO_HPP__

#include "trs.hpp"
#include "vec3_cu.hpp"
#include "camera.hpp"

/**
  @class Gizmo
  @brief a Gizmo is a 3D object enabling the user to transform an a object of
  the scene

  This base class is used to represent a 3D object: the Gizmo. It can be
  displayed and grabbed with the mouse to enable the user moving objects into
  the scene.

  Usually this class is intended to be specialized to perform translation,
  rotation and scaling.

  One can set the gizmo global orientation and position in space using :
  @code
  set_frame();
  set_transfo();
  set_org();
  @endcode

  the method 'draw()' paint with openGL the gizmo given it's
  position/orientation (i.e attribute '_frame'). Hidding the gizmo and disabling
  its selection is done with show(bool state).

  Use case:
  @code
  Gizmo* giz = new Gizmo_rot();

  void mouse_click_on_pixel(int x, int y)
  {
      if( giz->select_constraint(cam, x, y) )
      {
          // Constraint has been selected
      }
  }

  void mouse_move_on_pixel(int x, int y)
  {
      // Get the transformation of the gizmo.
      // (expressed in local coordinates of the gizmo when clicked)
      TRS transfo = giz->slide(cam, x, y);

  }

  void mouse_release()
  {
      giz->reset_constraint();
  }
  @endcode

  @see Gizmo_scale Gizmo_rot Gizmo_trans Gizmo_trackball
*/
class Gizmo {
public:
    enum Gizmo_t { TRANSLATION, ROTATION, TRACKBALL, SCALE };

    struct Pix{ int x, y; };

    Gizmo() :
        _old_frame( Transfo::identity() ),
        _frame( Transfo::identity() ),
        _show(false)
    { }

    virtual ~Gizmo(){ }

    void copy(const Gizmo* obj){
        _frame = obj->_frame;
        _show  = obj->_show;
    }

    /// Set frame orientation (world coordinates)
    void set_frame(const Vec3_cu& fx, const Vec3_cu& fy, const Vec3_cu& fz){
        _frame.set_x( fx.normalized() );
        _frame.set_y( fy.normalized() );
        _frame.set_z( fz.normalized() );
    }

    /// Set frame orientation (world coordinates)
    void set_frame(const Mat3_cu& frame){
        _frame = Transfo(frame.normalized(), _frame.get_translation());
    }

    /// Sets the origin of the frame (world coordinates)
    void set_org(const Vec3_cu& pt){ _frame.set_translation(pt); }

    /// Set frame orientation and origin (world coordinates)
    /// @see set_org() set_frame()
    void set_transfo(const Transfo& tr) { _frame = tr.normalized(); }

    /// @return the current frame of the gizmo
    Transfo frame() const { return _frame; }

    /// @return the frame of the gizmo when a constraint has been selected
    Transfo old_frame() const { return _old_frame; }

    /// Draw the gizmo according to its orientation and origin.
    /// Selected constraint with 'select_constraint()' will be highlighted.
    /// drawing must be enabled with show()
    /// @see set_transfo() show() select_constraint() reset_constraint()
    virtual void draw(const Camera& cam) = 0;

    /// Disable the gizmo drawing and selection
    void show(bool state){ _show = state; }

    /// select a constraint (for instance the 'x' axis of translation)
    /// given a camera and a mouse position. This also updates '_old_frame'
    /// attribute given the current '_frame'
    /// @return true if a constraint has been selected
    virtual bool select_constraint(const Camera& cam, int px, int py) = 0;

    /// reset the selected constraint set by select_constraint(),
    /// this disable slide and the highlighting with draw
    virtual void reset_constraint() = 0;

    /// @brief slide point given the current selected constraint.
    /// Given a moving mouse position (px, py) we deduce the transformation made
    /// by the gizmo knowing the selected constraint and the old mouse position.
    /// usually we try to keep the mouse as close as possible under the gizmo
    /// @note to change the position of the gizmo when calling 'draw()' don't
    /// forget to update the transformation with a set_transfo()
    /// @return The transformation made by the gizmo in local coordinates
    /// of the gizmo frame (when clicked on i.e _old_frame).
    /// @see select_constraint() reset_constraint() set_transfo()
    virtual TRS slide(const Camera& cam, int px, int py) = 0;

    Transfo _old_frame; ///< frame of the gizmo when selecting a constraint
protected:
    Transfo _frame;     ///< orientation and position of the gizmo
    bool    _show;      ///< do we draw the gizmo
};


#endif // GIZMO_HPP__
