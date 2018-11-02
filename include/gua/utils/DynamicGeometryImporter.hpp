/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
 *                                                                            *
 * This program is free software: you can redistribute it and/or modify it    *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option)  *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
 * for more details.                                                          *
 *                                                                            *
 * You should have received a copy of the GNU General Public License along    *
 * with this program. If not, see <http://www.gnu.org/licenses/>.             *
 *                                                                            *
 ******************************************************************************/

#ifndef GUA_DYNAMIC_GEOMETRY_IMPORTER_HPP
#define GUA_DYNAMIC_GEOMETRY_IMPORTER_HPP

// guacamole headers
#include <gua/config.hpp>
#include <gua/platform.hpp>

// external headers
#include <scm/gl_core.h>
#include <scm/core/math/quat.h>

#include <vector>


namespace gua {


class TransformNode;

using IndexTriplet = std::tuple<int,int,int>;

struct DynamicGeometryObject {

  DynamicGeometryObject(unsigned int max_geometry_vertices = 10000) {
    vertex_position_database.reserve(max_geometry_vertices);
    vertex_color_database.reserve(max_geometry_vertices);
    vertex_thickness_database.reserve(max_geometry_vertices);
    vertex_normal_database.reserve(max_geometry_vertices);

  }

  std::vector<scm::math::vec3f> vertex_position_database;
  std::vector<scm::math::vec4f> vertex_color_database;
  std::vector<float> 			vertex_thickness_database;
  std::vector<scm::math::vec3f> vertex_normal_database;

  //not used in the first version of the importer
  std::vector<IndexTriplet> vertex_attribute_ids;
};

using NamedDynamicGeometryObject = std::pair<std::string, DynamicGeometryObject>;

/**
 * @brief holds vertex information of one dynamic geometry
 */
class GUA_DLL DynamicGeometryImporter {

  friend class DynamicGeometry;
  
  public:

    void create_empty_dynamic_geometry(std::string const& empty_dynamic_geometry_name);

    bool parsing_successful() const;	
    
    void read_file(std::string const& file_name);

    int num_parsed_dynamic_geometries() const;

	NamedDynamicGeometryObject parsed_dynamic_geometry_object_at(int dynamic_geometry_object_index) const;

  private:

  	bool parsing_successful_ = false;

  	int num_parsed_dynamic_geometries_ = 0;

  	std::vector<NamedDynamicGeometryObject> parsed_dynamic_geometry_objects_;



};


}

#endif //GUA_DYNAMIC_GEOMETRY_IMPORTER_HPP
