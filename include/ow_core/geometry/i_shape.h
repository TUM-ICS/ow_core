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

#ifndef OPEN_WALKER_CORE_GEOMETRY_ISHAPES_H
#define OPEN_WALKER_CORE_GEOMETRY_ISHAPES_H

#include <ow_core/types.h>
#include <visualization_msgs/Marker.h>

// the namespace for the project
namespace ow
{
  /**
   * @brief The Base class for all shapes
   * 
   */
  class ShapeBase
  {
  public:
    /**
     * @brief Construct a new Shape Base object
     * 
     */
    ShapeBase()
    {
    }

    /**
     * @brief Destroy the Shape Base object
     * 
     */
    virtual ~ShapeBase()
    {
    }

    /**
     * @brief check the collision between shape and point
     * 
     * @param position 
     * @param min_dist 
     * @return true 
     * @return false 
     */
    virtual bool checkCollision(const ow::Vector2 &point, ow::Scalar min_dist) const = 0;

    /**
     * @brief return the minimum distance between shape and point
     * 
     * @param position 
     * @return ow::Scalar 
     */
    virtual ow::Scalar minimumDistance(const ow::Vector2 &position) const = 0;

    /**
     * @brief return the minimum distance between shape and polygon
     * 
     * @param polygon 
     * @return ow::Scalar 
     */
    virtual ow::Scalar minimumDistance(const ow::Points2d &polygon) const = 0;

    /**
     * @brief return the closest point on the shape
     * 
     * @param point 
     * @return const ow::Vector2 
     */
    virtual const ow::Vector2 closestPoint(const ow::Vector2 &point) const = 0;

    /**
     * @brief fit the shape to given set of points
     * 
     * @param points 
     */
    virtual void fit(const ow::Points2d& points)
    {
    }

    /**
     * @brief convert to a ros visualization maker msg
     * 
     * @return visualization_msgs::Marker 
     */
    virtual visualization_msgs::Marker toMarkerMsg(
      ow::Scalar r = 0.0, 
      ow::Scalar g = 0.0, 
      ow::Scalar b = 0.0, 
      ow::Scalar a = 1.0,
      ow::Scalar scale = 0.05) = 0; 
  };

}

#endif