// class header
#include <gua/utils/DynamicGeometryImporter.hpp>
// guacamole headers
#include <gua/utils/Logger.hpp>
#include <gua/utils/ToGua.hpp>
// #include <gua/utils/Timer.hpp>

//external headers
#include <iostream>
#include <fstream>
#include <sstream>

namespace gua {


void DynamicGeometryImporter::
create_empty_dynamic_geometry(std::string const& empty_dynamic_geometry_name) {
  parsing_successful_ = true;
  num_parsed_dynamic_geometries_ = 1;
  
  parsed_dynamic_geometry_objects_.push_back( std::make_pair(empty_dynamic_geometry_name, DynamicGeometryObject()) );
}

void DynamicGeometryImporter::
read_file(std::string const& file_name) {
  parsing_successful_ = false;
  std::ifstream in_lob_file(file_name, std::ios::in );

  if(!in_lob_file.is_open()) {
    Logger::LOG_WARNING << "Could not open *.lob file for reading!" << std::endl;
  }

  std::string dynamic_geometry_buffer("");

  auto ltrim = [](std::string& string_to_trim) {
      string_to_trim.erase(string_to_trim.begin(), std::find_if(string_to_trim.begin(), string_to_trim.end(), [](int ch) {
          return !std::isspace(ch);
      }));
  };

  NamedDynamicGeometryObject* current_dynamic_geometry_object = nullptr;

  while(std::getline(in_lob_file, dynamic_geometry_buffer)) {
    
    //trim whitespace to the left of the string
    ltrim(dynamic_geometry_buffer);

    if(dynamic_geometry_buffer.empty()) {
      continue;
    }

    std::istringstream in_sstream(dynamic_geometry_buffer);
    char dynamic_geometry_prefix;

    in_sstream >> dynamic_geometry_prefix;

    //create new object
    if('o' == dynamic_geometry_prefix) {
      std::string new_object_name = "";

      in_sstream >> new_object_name;
      parsed_dynamic_geometry_objects_.push_back( std::make_pair(new_object_name, DynamicGeometryObject()) );

      current_dynamic_geometry_object = &(parsed_dynamic_geometry_objects_).back();

      ++num_parsed_dynamic_geometries_;
    } else {
      float* float_attributes_to_parse = nullptr;
      switch(dynamic_geometry_prefix) {
        case 'v':
          float_attributes_to_parse = new float[3];
          for(int position_idx = 0; position_idx < 3; ++position_idx) {
            in_sstream >> float_attributes_to_parse[position_idx];
          }
          current_dynamic_geometry_object
            ->second.vertex_position_database.emplace_back( float_attributes_to_parse[0], 
                                                            float_attributes_to_parse[1], 
                                                            float_attributes_to_parse[2]);
          delete[] float_attributes_to_parse;
          break;

        case 'c':
          float_attributes_to_parse = new float[4];
          for(int color_idx = 0; color_idx < 4; ++color_idx) {
            in_sstream >> float_attributes_to_parse[color_idx];
          }
          current_dynamic_geometry_object
            ->second.vertex_color_database.emplace_back( float_attributes_to_parse[0], 
                                                         float_attributes_to_parse[1], 
                                                         float_attributes_to_parse[2],
                                                         float_attributes_to_parse[3]);
          delete[] float_attributes_to_parse;
          break;

        // case 'u':
        //   float_attributes_to_parse = new float[4];
        //   for(int color_idx = 0; color_idx < 4; ++color_idx) {
        //     in_sstream >> float_attributes_to_parse[color_idx];
        //   }
        //   current_dynamic_geometry_object
        //     ->second.vertex_uv_texture_database.emplace_back( float_attributes_to_parse[0], 
        //                                                       float_attributes_to_parse[1]);
        //   delete[] float_attributes_to_parse;
        //   break;

        case 't':
          float_attributes_to_parse = new float;
          in_sstream >> float_attributes_to_parse[0];
          
          current_dynamic_geometry_object
            ->second.vertex_thickness_database.emplace_back( float_attributes_to_parse[0]);
          delete float_attributes_to_parse;
          break;

        case 'n':
          float_attributes_to_parse = new float[3];
          for(int normal_idx = 0; normal_idx < 3; ++normal_idx) {
            in_sstream >> float_attributes_to_parse[normal_idx];
          }
          current_dynamic_geometry_object
            ->second.vertex_normal_database.emplace_back( float_attributes_to_parse[0], 
                                                          float_attributes_to_parse[1], 
                                                          float_attributes_to_parse[2]);
          delete[] float_attributes_to_parse;
          break;

        case 's':
          Logger::LOG_WARNING << "*.lob-parser option 's' is not implemented yet" << std::endl;
          break;

        default:
          Logger::LOG_WARNING << "Unknown *.lob-parser option " << dynamic_geometry_prefix << std::endl;
      }
    }
  }

  for(auto& current_dynamic_geometry_object : parsed_dynamic_geometry_objects_) {

    if(current_dynamic_geometry_object.second.vertex_position_database.size() > 
       current_dynamic_geometry_object.second.vertex_color_database.size()) {
      current_dynamic_geometry_object.second.vertex_color_database.resize(
         current_dynamic_geometry_object.second.vertex_position_database.size(),
          scm::math::vec4(1.0f, 1.0f, 1.0f, 1.0f)
        );
    }

    if(current_dynamic_geometry_object.second.vertex_position_database.size() > 
       current_dynamic_geometry_object.second.vertex_thickness_database.size()) {
      current_dynamic_geometry_object.second.vertex_thickness_database.resize(
         current_dynamic_geometry_object.second.vertex_position_database.size(),
          1.0f
        );
    }

    if(current_dynamic_geometry_object.second.vertex_position_database.size() > 
       current_dynamic_geometry_object.second.vertex_normal_database.size()) {
      current_dynamic_geometry_object.second.vertex_normal_database.resize(
         current_dynamic_geometry_object.second.vertex_position_database.size(),
          scm::math::vec3(0.0f, 1.0f, 0.0f)
        );
    }
  }

  parsing_successful_ = true;
}

bool DynamicGeometryImporter::
parsing_successful() const {
  return parsing_successful_;
}

int DynamicGeometryImporter::
num_parsed_dynamic_geometries() const {
  return num_parsed_dynamic_geometries_;
}

NamedDynamicGeometryObject DynamicGeometryImporter::
parsed_dynamic_geometry_object_at(int dynamic_geometry_object_index) const {
  return parsed_dynamic_geometry_objects_[dynamic_geometry_object_index];
}

} // namespace gua