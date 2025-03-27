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

#ifndef OPEN_WALKER_CORE_GEOMETRY_GEOMETRY_2D
#define OPEN_WALKER_CORE_GEOMETRY_GEOMETRY_2D

#include <ow_core/types.h>
#include <ow_core/math/cross_product.h>

// the namespace for the project
namespace ow
{

  /**
   * @brief cross product beetween two 2d vectors
   * 
   * @param p1 
   * @param p2 
   * @return ow::Scalar 
   */
  inline ow::Scalar angle_sign(const ow::Point2d &p1, const ow::Point2d &p2)
  {
    return p1.x() * p2.y() - p2.x() * p1.y();
  }

  /**
 * @brief distance between two points
 * 
 * @param p1 
 * @param p2 
 * @return ow::Scalar 
 */
  inline ow::Scalar distance_points(const ow::Point2d &p1, const ow::Point2d &p2)
  {
    return (p1 - p2).norm();
  }

  /**
   * @brief distance point p to parametric line: w^T + w0 = 0
   * 
   * parameter vector has the form [b, w1, w2]
   * 
   * @param p 
   * @param w
   * @return ow::Scalar 
   */
  inline ow::Scalar distance_point_line(const ow::Point2d &p, const ow::Vector3 &w)
  {
    return std::abs(w.tail(2).dot(p) + w[0]) / w.tail(2).norm();
  }

  /**
   * @brief compute the closest point to p on parametric line: w^T + w0 = 0 
   * 
   * @param p 
   * @param w 
   * @return ow::Point2d 
   */
  inline ow::Point2d closest_point_on_line(const ow::Point2d &p, const ow::Vector3 &w)
  {
    ow::Scalar d = w[1] * w[1] + w[2] * w[2];

    if (d > 0.0)
    {
      ow::Point2d cp;
      cp[0] = (w[2] * w[2] * p[0] - w[1] * w[2] * p[1] - w[1] * w[0]) / d;
      cp[1] = (-w[1] * w[2] * p[0] + w[1] * w[1] * p[1] - w[2] * w[0]) / d;
      return cp;
    }

    // error
    return p;
  }

  /**
   * @brief distance point p to line defined by start point s and end point e
   * 
   * @param p 
   * @param s 
   * @param e 
   * @return ow::Scalar 
   */
  inline ow::Scalar distance_point_line(
      const ow::Point2d &p,
      const ow::Point2d &s,
      const ow::Point2d &e)
  {
    ow::Point2d d = e - s;
    return (d.y() * p.x() - d.x() * p.y() + e.x() * s.y() - e.y() * s.x()) / d.norm();
  }

  /**
   * @brief projection of point p to the line defines by start point s and end point e
   * 
   * @param p 
   * @param s 
   * @param e 
   * @return ow::Point2d 
   */
  inline ow::Point2d closest_point_on_line_segment(
      const ow::Point2d &p, const ow::Point2d &s, const ow::Point2d &e,
      ow::Scalar *distance = NULL)
  {
    ow::Point2d cp;
    ow::Point2d d = e - s;
    ow::Scalar sq_n = d.squaredNorm();

    if (sq_n == 0)
    {
      if (distance)
      {
        *distance = 0.0;
      }
      return s; // no line segment
    }

    ow::Scalar u = ((p.x() - s.x()) * d.x() + (p.y() - s.y()) * d.y()) / sq_n;
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
    if (distance)
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
      const ow::Point2d &p, const ow::Point2d &s, const ow::Point2d &e)
  {
    return (p - closest_point_on_line_segment(p, s, e)).norm();
  }

  /**
 * @brief checks if two line segments (s1, e1) and (s2, e2) intersect
 * 
 * See http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
 * 
 * @param s1 
 * @param e1 
 * @param s2 
 * @param e2 
 * @param intersection 
 * @return true 
 * @return false 
 */
  inline bool check_line_segments_intersection(
      const ow::Point2d &s1, const ow::Point2d &e1,
      const ow::Point2d &s2, const ow::Point2d &e2,
      ow::Point2d *intersection = NULL)
  {
    ow::Scalar s_numer, t_numer, denom, t;
    ow::Point2d line1 = e1 - s1;
    ow::Point2d line2 = e2 - s2;

    denom = line1.x() * line2.y() - line2.x() * line1.y();
    if (denom == 0)
    {
      return false; // Collinear
    }

    bool is_denom_pos = denom > 0;
    ow::Point2d aux = s1 - s2;

    s_numer = line1.x() * aux.y() - line1.y() * aux.x();
    if ((s_numer < 0) == is_denom_pos)
    {
      return false; // No collision
    }

    t_numer = line2.x() * aux.y() - line2.y() * aux.x();
    if ((t_numer < 0) == is_denom_pos)
    {
      return false; // No collision
    }

    if (((s_numer > denom) == is_denom_pos) || ((t_numer > denom) == is_denom_pos))
    {
      return false; // No collision
    }

    // Otherwise collision detected
    t = t_numer / denom;
    if (intersection)
    {
      *intersection = s1 + t * line1;
    }
    return true;
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
      const ow::Point2d &s1, const ow::Point2d &e1,
      const ow::Point2d &s2, const ow::Point2d &e2)
  {
    if (check_line_segments_intersection(s1, e1, s2, e2))
    {
      return 0;
    }

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
   * @return ow::Point2d 
   */
  inline ow::Point2d closest_point_on_line_segment(
      const ow::Point2d &s1, const ow::Point2d &e1,
      const ow::Point2d &s2, const ow::Point2d &e2,
      ow::Scalar *distance = NULL)
  {
    ow::Point2d closest_point;
    if (check_line_segments_intersection(s1, e1, s2, e2, &closest_point))
    {
      if (distance)
      {
        *distance = 0;
      }
      return closest_point;
    }

    ow::Scalar d1, d2;
    ow::Point2d p;
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
      const ow::Point2d &s1, const ow::Point2d &e1,
      const ow::Point2d &s2, const ow::Point2d &e2)
  {
    ow::Point2d p1 = (e1 - s1).normalized();
    ow::Point2d p2 = (e2 - s2).normalized();
    return p1.dot(p2);
  }

  /**
   * @brief distance of point p to polygon defined by vertice points
   * 
   * @param p 
   * @param vertices 
   * @return ow::Scalar 
   */
  inline ow::Scalar distance_point_polygon(
      const ow::Point2d &p, const Points2d &vertices)
  {
    if (vertices.cols() == 1)
    {
      return (p - vertices.col(0)).norm();
    }

    // check each edge
    ow::Scalar dmin = std::numeric_limits<ow::Scalar>::max();
    for (size_t i = 0; i < vertices.cols() - 1; ++i)
    {
      ow::Scalar d = distance_point_line_segment(p, vertices.col(i), vertices.col(i+1));
      dmin = d < dmin ? d : dmin;
    }
    // if not a line check edge back to front
    if (vertices.cols() > 2)
    {
      ow::Scalar d = distance_point_line_segment(p, vertices.rightCols(1), vertices.col(0));
      dmin = d < dmin ? d : dmin;
    }
    return dmin;
  }

  /**
   * @brief distance line segment (s, e) to polygon defined by vertice points
   * 
   * @param s 
   * @param e 
   * @param vertices 
   * @return ow::Scalar 
   */
  inline ow::Scalar distance_line_polygon(
      const ow::Point2d &s, const ow::Point2d &e,
      const Points2d &vertices)
  {
    if (vertices.cols() == 1)
    {
      return distance_point_line_segment(vertices.col(0), s, e);
    }

    ow::Scalar dmin = std::numeric_limits<ow::Scalar>::max();
    for (size_t i = 0; i < vertices.cols() - 1; ++i)
    {
      ow::Scalar d = distance_segment_segment(s, e, vertices.col(i), vertices.col(i+1));
      dmin = d < dmin ? d : dmin;
    }
    if (vertices.cols() > 2)
    {
      ow::Scalar d = distance_segment_segment(s, e, vertices.rightCols(1), vertices.col(0));
      dmin = d < dmin ? d : dmin;
    }
    return dmin;
  }

  /**
   * @brief distance between two polygons defined by points in vertices1 and vertices2
   * 
   * @param vertices1 
   * @param vertices2 
   * @return ow::Scalar 
   */
  inline ow::Scalar distance_polygon_polygon(
      const Points2d &vertices1,
      const Points2d &vertices2)
  {
    if (vertices1.size() == 1)
    {
      return distance_point_polygon(vertices1.col(0), vertices2);
    }

    ow::Scalar dmin = std::numeric_limits<ow::Scalar>::max();
    for (size_t i = 0; i < vertices1.size() - 1; ++i)
    {
      ow::Scalar d = distance_line_polygon(vertices1.col(i), vertices1.col(i+1), vertices2);
      dmin = d < dmin ? d : dmin;
    }
    if (vertices1.size() > 2)
    {
      ow::Scalar d = distance_line_polygon(vertices1.rightCols(1), vertices1.col(0), vertices2);
      dmin = d < dmin ? d : dmin;
    }
    return dmin;
  }

  /**
   * @brief compute the closest point on polygon defined by vertices from point p
   * 
   * @param point 
   * @param vertices 
   * @param distance 
   * @return ow::Point2d 
   */
  inline ow::Point2d closest_point_on_polygon(
      const ow::Point2d &p, const Points2d &vertices,
      ow::Scalar *distance = NULL)
  {
    if (vertices.cols() == 1)
    {
      return vertices.col(0);
    }

    // check each edge
    ow::Scalar dmin = std::numeric_limits<ow::Scalar>::max();
    ow::Point2d closest_pt;

    for (size_t i = 0; i < vertices.cols() - 1; ++i)
    {
      ow::Point2d pt = closest_point_on_line_segment(p, vertices.col(i), vertices.col(i-1));
      ow::Scalar d = (pt - p).norm();
      if (d < dmin)
      {
        closest_pt = pt;
        dmin = d;
      }
    }
    // if not a line check edge back to front
    if (vertices.cols() > 2)
    {
      ow::Point2d pt = closest_point_on_line_segment(p, vertices.rightCols(1), vertices.col(0));
      ow::Scalar d = (pt - p).norm();
      if (d < dmin)
      {
        closest_pt = pt;
        dmin = d;
      }
    }
    if (distance)
    {
      *distance = dmin;
    }
    return closest_pt;
  }

  /**
   * @brief Compute the union polygon
   * 
   * This function assumes that the polygons are not overlapping
   * 
   * @param vertices1 
   * @param vertices2 
   * @return std::vector<ow::Point2d> 
   */
  inline Points2d union_polygon_no_overlap(
      const Points2d &vertices1,
      const Points2d &vertices2)
  {
    std::size_t n = vertices1.cols() + vertices2.cols();
    ow::MatrixX vertices(2, n);
    vertices << vertices1, vertices2;

    // subtract corner to put everything in fist quadrant
    ow::Point2d p_min = vertices.rowwise().minCoeff();
    vertices.colwise() -= p_min;

    // find the lower left point
    ow::MatrixX::Index idx_start;
    vertices.colwise().norm().minCoeff(&idx_start);

    std::vector<bool> visited(n, false);
    Points2d vertices_union;

    ow::MatrixX::Index idx_cur = idx_start;
    ow::MatrixX::Index idx_next = -1;
    ow::MatrixX::Index size = 0;
    ow::Point2d v_cur, e, e_cur;
    ow::Scalar a, a_max;

    v_cur = vertices.col(idx_cur);

    if(v_cur.norm() > 1e-8)
      e_cur = v_cur.normalized();
    else
      e_cur << 1.0, 0.0;

    a_max = -2.0;
    while(!visited[idx_start])
    {
      for(std::size_t i = 0; i < n; ++i)
      {
        if(i != idx_cur && !visited[i])
        {
          // next possible edge
          e = (vertices.col(i) - v_cur);

          // cosine distance [-1, 1]
          a = e_cur.dot(e)/(e.norm() * e_cur.norm());

          // first iteration find the minimum instead of maximum
          if(size == 0)
          {
            a = 1.0 - a; // [-2, 0]
          }

          // select pos moment and max angle vertice
          if(a > a_max)
          {
            a_max = a;
            idx_next = i;
          }
        }
      }

      // step to next vertice, remember edge
      e_cur = vertices.col(idx_next) - v_cur;

      v_cur = vertices.col(idx_next);
      idx_cur = idx_next;

      // update solution
      vertices_union.conservativeResize(Eigen::NoChange, size+1);
      vertices_union.col(size) = v_cur + p_min;
      size++;

      visited[idx_cur] = true;
      a_max = -1.0;
    }
    return vertices_union;
  } 

  /**
   * @brief check intersection between point p and the rectable defined by corners
   * rect_min and rect_max
   * 
   * @param rect_min 
   * @param rect_max 
   * @param p 
   * @return true 
   * @return false 
   */
  inline bool point_rect_intersect(
      const ow::Point2d &rect_min, const ow::Point2d &rect_max, const ow::Point2d &p)
  {
    return (p.x() > rect_min.x() && p.x() < rect_max.x() &&
            p.y() > rect_min.y() && p.y() < rect_max.y());
  }

  /**
   * @brief check if point intersects polygon
   * 
   * Note: assumes vertices are ordered in counter clockwise manner 
   * 
   * @param vertices 
   * @param p 
   * @return true 
   * @return false 
   */
  inline bool point_polygon_intersect(const ow::Points2d& vertices, const ow::Point2d &p)
  {
    bool c = false;

    // check if inside
    size_t i, j;
    for (i = 0, j = vertices.cols() - 1; i < vertices.cols(); j = i++)
    {
      if (((vertices.col(i).y() > p.y()) != (vertices.col(j).y() > p.y())) &&
          (p.x() < (vertices.col(j).x() - vertices.col(i).x()) * (p.y() - vertices.col(i).y()) / (vertices.col(j).y() - vertices.col(i).y()) + vertices.col(i).x()))
      {
        c = !c;
      }
    }
    return c > 0;
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
  inline bool point_circle_intersect(
      const ow::Point2d &center, ow::Scalar radius, const ow::Point2d &p)
  {
    return (center - p).norm() < radius;
  }

} // namespace ow

#endif
