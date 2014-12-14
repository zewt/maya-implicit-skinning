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
#ifndef TRACKBALL_HPP__
#define TRACKBALL_HPP__

/**
  @class TrackBall
  @brief Handles trackball movements

  Tracball are intuitive rotations given a mouse movements.
  Given a clicked point and a second moving point on the screen we compute a
  rotation that gives the user the impression of dragging the selected point on
  a 3D ball with the mouse.

  Use case:
  @code
  Point_cu trackball_center3D;
  Point_cu trackball_center2D = cam.project(trackball_center3D);

  TrackBall ball(cam.width(), cam.height(), trackball_center2D.x, trackball_center2D.y, 1.f);

  void mouse_click_on_pixel(int x, int y)
  {
      // Sets the point to be dragged on the trackball (window coords)
      ball.set_picked_point(x, y);
  }

  void mouse_move_on_pixel(int x, int y)
  {
      // Compute the rotation made by the trackball according to the new mouse
      // position
      Vec3_cu eye_axis;
      float angle;
      ball.roll( (float*)&eye_axis, angle, x, y );

      // Axis is in eye coordinates we transform it into world coords
      Vec3_cu world_axis = cam.get_eye_transfo().fast_invert() * eye_axis;
      // Rotation of the trackball in world coordinates
      Transfo rot = Transfo::rotate(trackball_center3D, world_axis, angle);
  }
  @endcode

  @note The trackball radius must be large enough. Usually movements made by
  the user mouse will cover half the viewport. Small radius are harder to
  controll and movements of the trackball can even become very shaky/random
  (Maybe because of numerical instabilities)
*/
class TrackBall {
public:

    /// @param width : window width in pixels
    /// @param height : window height in pixels
    /// @param cx : center of the trackball in window coordinates.
    /// @param cy : center of the trackball in window coordinates.
    /// @param radius : radius of the trackball
    /// @note usually we project on the window the 3D point that represent the
    /// center of the tracball to get 'cx' and 'cy'
    TrackBall(int width, int height, float cx, float cy, float radius) :
        _width(width),
        _height(height),
        _cx(cx),
        _cy(cy),
        _rad(radius)
    { }

    /// Sets the point to be dragged on the trackball,
    /// the point (x, y) is in window coordinates
    void set_picked_point(float x, float y) {
        _pick_x = x; _pick_y = y;
    }


    /// Compute the trackball transformation.
    /// Given the internal parameters of the trackball and a moving point
    /// p(px, py) we deduce the rotation done by the trackball.
    /// p is in window coordinates.
    /// @param axis The axis of rotation 'axis' is expressed in eye coordinates,
    /// you'll have to multiply it by the inverse of the eye matrix to get its
    /// world coordinates.
    /// @param phi : the angle of rotation made by the trackball (radian)
    /// @see set_picked_point() roll()
    void roll(float a[3], float& phi, float px, float py) const;

    /// Compute a trackball transformation. Given a reference point p1(p1x, p1y)
    /// and a moving point p2(p2x, p2y) compute the angle and axis of rotation
    /// made by the dragged trackball.
    /// The trackball is assumed to be at the center of the window. Points p1
    /// and p2 are in normalized window coordinates (their coordinates must lie
    /// between [-1 1] ). To consider a trackball not centered at the origin
    /// translate p1 and p2
    /// @param axis The axis of rotation 'axis' is expressed in eye coordinates,
    /// you'll have to multiply it by the inverse of the eye matrix to get its
    /// world coordinates.
    /// @param rad : radius of the trackball for a normalized window coordinates
    /// (between [-1 1])
    /// @param phi : the angle of rotation made by the trackball
    /// @note it is often more easier to use the none static version
    /// of this method
    static void roll(float axis[3], float& phi, float rad, float p1x, float p1y, float p2x, float p2y);

private:
    int _width;    ///< width of the window
    int _height;   ///< height of the window
    float _pick_x; ///< picked point x in window coords
    float _pick_y; ///< picked point y in window coords
    float _cx;     ///< trackball center x in window coords
    float _cy;     ///< trackball center y in window coords
    float _rad;    ///< trackball radius
};

#endif // TRACKBALL_HPP__
