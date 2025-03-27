/*! \file
 *
 * \author Emmanuel Dean-Leon
 * \author Florian Bergner
 * \author J. Rogelio Guadarrama-Olvera
 * \author Simon Armleder
 * \author Gordon Cheng
 *
 * \version 0.1
 * \date 22.03.2021
 *
 * \copyright Copyright 2020 Institute for Cognitive Systems (ICS),
 *    Technical University of Munich (TUM)
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 * #### Acknowledgment
 *  This project has received funding from the European Union‘s Horizon 2020
 *  research and innovation programme under grant agreement No 732287.
 */

#ifndef OPEN_WALKER_CORE_GEOMETRY_GEOMETRY
#define OPEN_WALKER_CORE_GEOMETRY_GEOMETRY

#include <ow_core/types.h>

// the namespace for the project
namespace ow
{

  /**
   * @brief distance between two points
   * 
   * @param p1 
   * @param p2 
   * @return ow::Scalar 
   */
  inline ow::Scalar distance_points(const ow::Vector3 &p1, const ow::Vector3 &p2)
  {
    return (p1 - p2).norm();
  }

  /**
   * @brief distance point p to parametric line: w^T*x + w0 = 0
   * 
   * parameter vector has the form [b, w1, w2, w3]
   * 
   * @param p 
   * @param w
   * @return ow::Scalar 
   */
  inline ow::Scalar distance_point_line(const ow::Vector3 &p, const ow::Vector4 &w)
  {
    return std::abs(w.tail(3).dot(p) + w[0]) / w.tail(3).norm();
  }

  /**
   * @brief compute the closest point to p on parametric line: w^T + w0 = 0 
   * 
   * @param p 
   * @param w 
   * @return ow::Vector3 
   */
  inline ow::Vector3 closest_point_on_line(const ow::Vector3 &p, const ow::Vector4& w)
  {
    return p - distance_point_line(p, w)*w.tail(3);
  }

  /**
   * @brief distance point p to line defined by start point s and end point e
   * 
   * @param p 
   * @param s 
   * @param e 
   * @return ow::Scalar 
   */
  inline ow::Scalar distance_point_line(const ow::Vector3 &p, const ow::Vector3 &s, const ow::Vector3 &e)
  {
    ow::Vector3 d = (e - s).normalized();
    return d.cross(p).norm();
  }

  /**
   * @brief projection of point p to the line defines by start point s and end point e
   * 
   * @param p 
   * @param s 
   * @param e 
   * @return ow::Vector3 
   */
  inline ow::Vector3 closest_point_on_line_segment(
      const ow::Vector3 &p, const ow::Vector3 &s, const ow::Vector3 &e,
      ow::Scalar *distance = NULL)
  {
    ow::Vector3 cp;
    ow::Vector3 d = e - s;
    ow::Scalar sq_n = d.squaredNorm();

    if (sq_n == 0)
    {
      if(distance)
      {
        *distance = 0.0;
      }
      return s; // no line segment
    }

    ow::Scalar u = (p - s).dot(d) / sq_n;
    if (u <= 0)
    {
      cp = s;
    }
    else if (u >= 1)
    {
      cp = e;
    }
    else
    {
      cp = s + u * d;
    }
    if(distance)
    {
      *distance = (p - cp).norm();
    }
    return cp;
  }

  /**
   * @brief distance between point p and the line defines by start point s and end point e
   * 
   * @param p 
   * @param s 
   * @param e 
   * @return ow::Scalar 
   */
  inline ow::Scalar distance_point_line_segment(
      const ow::Vector3 &p, const ow::Vector3 &s, const ow::Vector3 &e)
  {
    return (p - closest_point_on_line_segment(p, s, e)).norm();
  }

  /**
   * @brief distance between two line segments
   * 
   * @param s1 
   * @param e1 
   * @param s2 
   * @param e2 
   * @return ow::Scalar 
   */
  inline ow::Scalar distance_segment_segment(
      const ow::Vector3 &s1, const ow::Vector3 &e1,
      const ow::Vector3 &s2, const ow::Vector3 &e2)
  {
    // check all 4 combinations
    ow::Scalar distances[4];
    distances[0] = distance_point_line_segment(s1, s2, e2);
    distances[1] = distance_point_line_segment(e1, s2, e2);
    distances[2] = distance_point_line_segment(s2, s1, e1);
    distances[3] = distance_point_line_segment(e2, s1, e1);

    // return minimum
    ow::Scalar d_min = std::numeric_limits<ow::Scalar>::max();
    for (size_t i = 0; i < 4; ++i)
    {
      d_min = distances[i] < d_min ? distances[i] : d_min;
    }
    return d_min;
  }

  /**
   * @brief clostest point on line segment (s2,e2) mesasured between two given line segments (s1,e1), (s2,e2)
   * 
   * @param s1 
   * @param e1 
   * @param s2 
   * @param e2 
   * @param distance 
   * @return ow::Vector3 
   */
  inline ow::Vector3 closest_point_on_line_segment(
      const ow::Vector3 &s1, const ow::Vector3 &e1,
      const ow::Vector3 &s2, const ow::Vector3 &e2,
      ow::Scalar *distance = NULL)
  {
    ow::Vector3 closest_point;

    ow::Scalar d1, d2;
    ow::Vector3 p;
    closest_point = closest_point_on_line_segment(s1, s2, e2);
    d1 = (s1 - closest_point).norm();
    p = closest_point_on_line_segment(e1, s2, e2);
    d2 = (e1 - p).norm();

    if (d1 < d2)
    {
      if (distance)
      {
        *distance = d1;
      }
      return closest_point;
    }
    else
    {
      if (distance)
      {
        *distance = d2;
      }
      return p;
    }
  }

  /**
   * @brief scalar product between line segments ( perpendicular = 0, collinear == 1 )
   * 
   * @param s1 
   * @param e1 
   * @param s2 
   * @param e2 
   * @return ow::Scalar 
   */
  inline ow::Scalar colinearity_segment_segment(
    const ow::Vector3 &s1, const ow::Vector3 &e1, 
    const ow::Vector3 &s2, const ow::Vector3 &e2)
  {
    ow::Vector3 p1 = (e1 - s1).normalized();
    ow::Vector3 p2 = (e2 - s2).normalized();
    return p1.dot(p2);
  }

  /**
   * @brief check intersection between point p and the rectable defined by corners
   * rect_min and rect_max
   * 
   * @param cube_min 
   * @param cube_max 
   * @param p 
   * @return true 
   * @return false 
   */
  inline bool point_cube_intersect(
      const ow::Vector3 &cube_min, const ow::Vector3 &cube_max, const ow::Vector3 &p)
  {
    return (p.x() > cube_min.x() && p.x() < cube_max.x() &&
            p.y() > cube_min.y() && p.y() < cube_max.y() && 
            p.z() > cube_min.z() && p.z() < cube_max.z());
  }

  /**
   * @brief check intersection between point p and circle (center, radius)
   * 
   * @param center 
   * @param radius 
   * @param p 
   * @return true 
   * @return false 
   */
  inline bool point_sphere_intersect(
      const ow::Vector3 &center, ow::Scalar radius, const ow::Vector3 &p)
  {
    return (center - p).norm() < radius;
  }
  
};

#endif