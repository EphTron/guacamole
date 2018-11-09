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

// class header
#include <gua/renderer/DynamicTriangleResource.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/node/DynamicTriangleNode.hpp>
#include <gua/utils/Logger.hpp>

#include <scm/gl_core/constants.h>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

DynamicTriangleResource::DynamicTriangleResource()
    : kd_tree_(), dynamic_triangle_(), vertex_rendering_mode_(scm::gl::PRIMITIVE_TRIANGLE_LIST), clean_flags_per_context_() {
    // : kd_tree_(), dynamic_triangle_(), vertex_rendering_mode_(scm::gl::PRIMITIVE_LINE_LIST), clean_flags_per_context_() {
  compute_bounding_box();
}

////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////

void DynamicTriangleResource::compute_bounding_box() {

  //if (dynamic_triangle_.num_occupied_vertex_slots > 0) {
    bounding_box_ = math::BoundingBox<math::vec3>();

    if(0 == dynamic_triangle_.num_occupied_vertex_slots) {
      bounding_box_.expandBy(math::vec3{-0.5, -0.5, -0.5 });
      bounding_box_.expandBy(math::vec3{ 0.5,  0.5,  0.5});      
    }
    if(1 == dynamic_triangle_.num_occupied_vertex_slots) {
       bounding_box_.expandBy(math::vec3{dynamic_triangle_.positions[0] - 0.0001f });
       bounding_box_.expandBy(math::vec3{dynamic_triangle_.positions[0] + 0.0001f });
    } else {
      for (int v(0); v < dynamic_triangle_.num_occupied_vertex_slots; ++v) {
        bounding_box_.expandBy(math::vec3{dynamic_triangle_.positions[v]});
      }
    }
  //}
}

////////////////////////////////////////////////////////////////////////////////

DynamicTriangleResource::DynamicTriangleResource(DynamicTriangle const& dynamic_triangle, bool build_kd_tree)
    : kd_tree_(), dynamic_triangle_(dynamic_triangle), clean_flags_per_context_() {
    
  compute_bounding_box();

  if (build_kd_tree) {
    //kd_tree_.generate(dynamic_triangle);
  }
}

////////////////////////////////////////////////////////////////////////////////

void DynamicTriangleResource::upload_to(RenderContext& ctx) const {
  std::lock_guard<std::mutex> lock(dynamic_triangle_update_mutex_);

/*
  if (dynamic_triangle_.vertex_reservoir_size == 0) {
    Logger::LOG_WARNING << "Unable to load DynamicTriangle! Has no vertex data." << std::endl;
    return;
  }
*/
 
  auto dynamic_triangle_iterator = ctx.dynamic_triangles.find(uuid());

  bool update_cached_dynamic_triangle{false};

  if((ctx.dynamic_triangles.end() == dynamic_triangle_iterator)) {
    ctx.dynamic_triangles[uuid()] = RenderContext::DynamicTriangle();
  }

  RenderContext::DynamicTriangle* dynamic_triangle_to_update_ptr = &ctx.dynamic_triangles[uuid()];

  if(update_cached_dynamic_triangle) {
    dynamic_triangle_to_update_ptr = &(dynamic_triangle_iterator->second);
  }

  dynamic_triangle_to_update_ptr->vertex_topology = scm::gl::PRIMITIVE_TRIANGLE_LIST;
  dynamic_triangle_to_update_ptr->vertex_reservoir_size = dynamic_triangle_.vertex_reservoir_size;
  dynamic_triangle_to_update_ptr->num_occupied_vertex_slots = dynamic_triangle_.num_occupied_vertex_slots;

  if(dynamic_triangle_to_update_ptr->current_buffer_size_in_vertices < dynamic_triangle_.vertex_reservoir_size) {
    dynamic_triangle_to_update_ptr->vertices =
        ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                         scm::gl::USAGE_DYNAMIC_DRAW,
                                         (dynamic_triangle_.vertex_reservoir_size) * sizeof(DynamicTriangle::TriVertex),
                                         0);

    dynamic_triangle_to_update_ptr->current_buffer_size_in_vertices = dynamic_triangle_.vertex_reservoir_size+3;
  } else {
    update_cached_dynamic_triangle = true;
  }

  if(dynamic_triangle_.vertex_reservoir_size != 0) {
    DynamicTriangle::TriVertex* data(static_cast<DynamicTriangle::TriVertex*>(ctx.render_context->map_buffer(
      dynamic_triangle_to_update_ptr->vertices, scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));

    dynamic_triangle_.copy_to_buffer(data);
    ctx.render_context->unmap_buffer(dynamic_triangle_to_update_ptr->vertices);
  
    dynamic_triangle_to_update_ptr->vertex_array = ctx.render_device->create_vertex_array(
         dynamic_triangle_.get_vertex_format(),
          {dynamic_triangle_to_update_ptr->vertices});

  }
  
  ctx.render_context->apply();
}

////////////////////////////////////////////////////////////////////////////////

void DynamicTriangleResource::draw(RenderContext& ctx) const {
  auto iter = ctx.dynamic_triangles.find(uuid());

  bool& clean_flag_for_context = clean_flags_per_context_[uuid()];

  if (iter == ctx.dynamic_triangles.end() || (!clean_flag_for_context) /*|| ctx_dirty_flag*/) {
    // upload to GPU if neccessary

    compute_consistent_normals();
    upload_to(ctx);
    iter = ctx.dynamic_triangles.find(uuid());

    clean_flag_for_context = true;
    //dirty_flags_per_context_[uuid()] = false;;
  }

  ctx.render_context->bind_vertex_array(iter->second.vertex_array);
  //ctx.render_context->bind_index_buffer(iter->second.indices, iter->second.indices_topology, iter->second.indices_type);
  ctx.render_context->apply_vertex_input();
  
  ctx.render_context->draw_arrays(iter->second.vertex_topology, 0, iter->second.num_occupied_vertex_slots);
  std::cout<< "draw call from resource " << std::endl;
}


////////////////////////////////////////////////////////////////////////////////


void DynamicTriangleResource::ray_test(Ray const& ray, int options,
                    node::Node* owner, std::set<PickResult>& hits) {

  //kd_tree_.ray_test(ray, dynamic_triangle_, options, owner, hits);
}

////////////////////////////////////////////////////////////////////////////////
/*
void DynamicTriangleResource::resolve_vertex_updates(RenderContext& ctx) {

  //TODO: PROTECT BY LINE STRIP UPDATE MUTEX

  for(auto& update_job_ptr : dynamic_triangle_update_queue_) {
    auto iter = ctx.dynamic_triangles.find(update_job_ptr->owner_uuid);
    auto ctx_dirty_flag_iter = dirty_flags_per_context_.find(uuid());
    bool ctx_dirty_flag = false;

    if (iter == ctx.dynamic_triangles.end() || ctx_dirty_flag) {
      dirty_flags_per_context_[uuid()] = false;;
    }

    update_job_ptr->execute(this);
  }
}
*/
////////////////////////////////////////////////////////////////////////////////

void DynamicTriangleResource::make_clean_flags_dirty() {
  for( auto& known_clean_flag : clean_flags_per_context_) {
    known_clean_flag.second = false;
  }
}

////////////////////////////////////////////////////////////////////////////////

void DynamicTriangleResource::compute_consistent_normals() const {
  std::lock_guard<std::mutex> lock(dynamic_triangle_update_mutex_);
  dynamic_triangle_.compute_consistent_normals();
}

////////////////////////////////////////////////////////////////////////////////

void DynamicTriangleResource::compile_buffer_string(std::string& buffer_string) {
  std::lock_guard<std::mutex> lock(dynamic_triangle_update_mutex_);
  dynamic_triangle_.compile_buffer_string(buffer_string);
};

////////////////////////////////////////////////////////////////////////////////

void DynamicTriangleResource::uncompile_buffer_string(std::string const& buffer_string) {
  std::lock_guard<std::mutex> lock(dynamic_triangle_update_mutex_);
  dynamic_triangle_.uncompile_buffer_string(buffer_string);
  make_clean_flags_dirty();
};


////////////////////////////////////////////////////////////////////////////////

void DynamicTriangleResource::push_vertex(DynamicTriangle::TriVertex const& in_vertex) {
  std::lock_guard<std::mutex> lock(dynamic_triangle_update_mutex_);

  if(dynamic_triangle_.push_vertex(in_vertex)) {
    if (dynamic_triangle_.num_occupied_vertex_slots > 0) {
        bounding_box_.expandBy(math::vec3{dynamic_triangle_.positions[dynamic_triangle_.num_occupied_vertex_slots-1]});
    }
    make_clean_flags_dirty();
  }

}

////////////////////////////////////////////////////////////////////////////////

void DynamicTriangleResource::pop_front_vertex() {
  std::lock_guard<std::mutex> lock(dynamic_triangle_update_mutex_);
  if(dynamic_triangle_.pop_front_vertex()){
    compute_bounding_box();
    make_clean_flags_dirty();
  }
}

////////////////////////////////////////////////////////////////////////////////

void DynamicTriangleResource::pop_back_vertex() {
  std::lock_guard<std::mutex> lock(dynamic_triangle_update_mutex_);
  if(dynamic_triangle_.pop_back_vertex()){
    compute_bounding_box();
    make_clean_flags_dirty();
  }
}

////////////////////////////////////////////////////////////////////////////////

void DynamicTriangleResource::clear_vertices() {
  std::lock_guard<std::mutex> lock(dynamic_triangle_update_mutex_);
  if(dynamic_triangle_.clear_vertices()) {
    compute_bounding_box();
    make_clean_flags_dirty();
  }
}

void DynamicTriangleResource::set_vertex_rendering_mode(scm::gl::primitive_topology const& render_mode){
  std::lock_guard<std::mutex> lock(dynamic_triangle_update_mutex_);
  vertex_rendering_mode_ = render_mode;
}

////////////////////////////////////////////////////////////////////////////////

void DynamicTriangleResource::forward_queued_vertices(std::vector<scm::math::vec3f> const& queued_positions,
                                                std::vector<scm::math::vec4f> const& queued_colors,
                                                std::vector<float> const& queued_thicknesses,
                                                std::vector<scm::math::vec2f> const& queued_uvs) {
  std::lock_guard<std::mutex> lock(dynamic_triangle_update_mutex_);
  dynamic_triangle_.forward_queued_vertices(queued_positions,
                                      queued_colors,
                                      queued_thicknesses,
                                      queued_uvs);
  compute_bounding_box();
  make_clean_flags_dirty();
}

////////////////////////////////////////////////////////////////////////////////

math::vec3 DynamicTriangleResource::get_vertex(unsigned int i) const {
  return math::vec3(
      dynamic_triangle_.positions[i].x, dynamic_triangle_.positions[i].y, dynamic_triangle_.positions[i].z);
}

////////////////////////////////////////////////////////////////////////////////

}
