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
#ifndef GIZMO2_HPP__
#define GIZMO2_HPP__

#include "trs.hpp"
#include "vec3_cu.hpp"
#include "vec2i_cu.hpp"
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
class Gizmo2 {
public:
    enum Gizmo2_t { TRANSLATION, ROTATION, TRACKBALL, SCALE };

    // =========================================================================
    /// @name Constructors
    // =========================================================================

    Gizmo2() :
        //_old_frame( Transfo::identity() ),
        _frame( Transfo::identity() ),
        _show(false)
    { }

    virtual ~Gizmo2(){ }

    void copy(const Gizmo2* obj){
        _frame = obj->_frame;
        _show  = obj->_show;
    }

    // =========================================================================
    /// @name Getters & Setters
    // =========================================================================

    /// Set frame orientation and origin (world coordinates) used
    /// to draw the gizmo and select it
    /// @see set_org() set_frame()
    void set_frame(const Transfo& tr) { _frame = tr.normalized(); }

    /// @return the current frame of the gizmo
    Transfo frame() const { return _frame; }

    // =========================================================================
    /// @name Drawing
    // =========================================================================

    /// Draw the gizmo according to its orientation and origin.
    /// Selected constraint with 'select_constraint()' will be highlighted.
    /// drawing must be enabled with show()
    /// @see set_transfo() show() select_constraint() reset_constraint()
    virtual void draw(const Camera& cam) = 0;

    /// Disable the gizmo drawing and selection
    void show(bool state){ _show = state; }

    // =========================================================================
    /// @name Gizmo selection
    // =========================================================================

    /// select a constraint (for instance the 'x' axis of translation)
    /// given a camera and a mouse position. This also updates '_old_frame'
    /// attribute given the current '_frame'
    /// @return true if a constraint has been selected
    virtual bool select_constraint(const Camera& cam, int px, int py) = 0;

    /// reset the selected constraint set by select_constraint(),
    /// this disable slide and the highlighting with draw
    virtual void reset_constraint() = 0;

    // =========================================================================
    /// @name Compute transformation
    // =========================================================================

    /// Sets starting position to compute the slide
    void slide_from( const Transfo start_frame, const Vec2i_cu& start_pix){
        _start_frame = start_frame;
        _start_pix   = start_pix;
    }

    /// @brief slide point given the current selected constraint.
    /// Given a moving mouse position (px, py) we deduce the transformation made
    /// by the gizmo knowing the selected constraint and the old mouse position.
    /// usually we try to keep the mouse as close as possible under the gizmo
    /// @note to change the position of the gizmo when calling 'draw()' don't
    /// forget to update the transformation with a set_transfo()
    /// @return The transformation made by the gizmo global coordinates
    /// @see select_constraint() reset_constraint() set_transfo()
    virtual TRS slide(const Camera& cam, int px, int py) = 0;

    // TODO: old_frame to be deleted
    //Transfo _old_frame; ///< frame of the gizmo when selecting a constraint

protected:
    Transfo  _start_frame;
    Vec2i_cu _start_pix;

    Transfo _frame;     ///< orientation and position of the gizmo
    bool    _show;      ///< do we draw the gizmo
};


#endif // GIZMO2_HPP__
