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
#include <gua/renderer/SkeletalAnimationRessource.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/node/SkeletalAnimationNode.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/math/BoundingBoxAlgo.hpp>


// external headers
#include <assimp/postprocess.h>
//#include <assimp/scene.h>

namespace gua {

SkeletalAnimationRessource::SkeletalAnimationRessource()
    : vertices_(), indices_(), vertex_array_(), upload_mutex_(), mesh_(){}

////////////////////////////////////////////////////////////////////////////////

SkeletalAnimationRessource::SkeletalAnimationRessource(Mesh const& mesh, std::shared_ptr<SkeletalAnimationDirector> animation_director, bool build_kd_tree)
: vertices_(),
  indices_(),
  vertex_array_(),
  upload_mutex_(),
  mesh_(mesh),
  animation_director_(animation_director)
{

  //TODO generate BBox and KDTree
  //if (mesh_->HasPositions()) {
  bounding_box_ = math::BoundingBox<math::vec3>();

  // without bone influence
  for (unsigned v(0); v < mesh_.num_vertices; ++v) {
    bounding_box_.expandBy(scm::math::vec3(
        mesh_.positions[v].x, mesh_.positions[v].y, mesh_.positions[v].z));
  }
  std::cout << "box dims" << bounding_box_.corners().first << " and " << bounding_box_.corners().second << std::endl;

  bone_boxes_ = std::vector<math::BoundingBox<math::vec3>>(100,math::BoundingBox<math::vec3>());

    // TODO
    /*if (build_kd_tree) {
      kd_tree_.generate(mesh_);
    }
  //}*/
}

////////////////////////////////////////////////////////////////////////////////
void SkeletalAnimationRessource::InitMesh(Mesh& mesh)
{   
  mesh.num_triangles = mesh_->mNumFaces;
  mesh.num_vertices = mesh_->mNumVertices; 
  
    // Reserve space in the vectors for the vertex attributes and indices
  mesh.positions.reserve(mesh.num_vertices);
  mesh.normals.reserve(mesh.num_vertices);
  mesh.texCoords.reserve(mesh.num_vertices);
  mesh.tangents.reserve(mesh.num_vertices);
  mesh.bitangents.reserve(mesh.num_vertices);
  mesh.bones.resize(mesh.num_vertices);
  mesh.indices.reserve(mesh.num_triangles);

  const scm::math::vec3 Zero3D(0.0f, 0.0f, 0.0f);
  
  // Populate the vertex attribute vectors
  for (uint i = 0 ; i < mesh_->mNumVertices ; i++) { // TODO catch: haspositions and hasnormals
    
    scm::math::vec3 pPos = scm::math::vec3(0.f, 0.f, 0.f);
    if(mesh_->HasPositions()) {
      pPos = scm::math::vec3(mesh_->mVertices[i].x,mesh_->mVertices[i].y,mesh_->mVertices[i].z);
    }

    scm::math::vec3 pNormal = scm::math::vec3(0.f, 0.f, 0.f);
    if(mesh_->HasNormals()) {
      pNormal = scm::math::vec3(mesh_->mNormals[i].x,mesh_->mNormals[i].y,mesh_->mNormals[i].z);
    }

    scm::math::vec3 pTexCoord = scm::math::vec3(0.0f,0.0f,0.0f);
    if(mesh_->HasTextureCoords(0)) {}
    {
      pTexCoord = scm::math::vec3(mesh_->mTextureCoords[0][i].x,mesh_->mTextureCoords[0][i].y,mesh_->mTextureCoords[0][i].z);
    }

    scm::math::vec3 pTangent = scm::math::vec3(0.f, 0.f, 0.f);
    scm::math::vec3 pBitangent = scm::math::vec3(0.f, 0.f, 0.f);
    if (mesh_->HasTangentsAndBitangents()) {
      pTangent = scm::math::vec3(mesh_->mTangents[i].x, mesh_->mTangents[i].y, mesh_->mTangents[i].z);

      pBitangent = scm::math::vec3(mesh_->mBitangents[i].x, mesh_->mBitangents[i].y, mesh_->mBitangents[i].z);
    }

    mesh.positions.push_back(pPos);
    mesh.normals.push_back(pNormal);
    mesh.bitangents.push_back(pBitangent);
    mesh.tangents.push_back(pTangent);
    mesh.texCoords.push_back(scm::math::vec2(pTexCoord[0], pTexCoord[1]));
  }

  LoadBones(mesh.bones);
  
  // Populate the index buffer
  for (uint i = 0 ; i < mesh_->mNumFaces ; i++) {
    const aiFace& Face = mesh_->mFaces[i];

    if(Face.mNumIndices != 3) {
      Logger::LOG_ERROR << "InitMesh - face doesnt have 3 vertices" << std::endl;
      assert(false);
    }

    mesh.indices.push_back(Face.mIndices[0]);
    mesh.indices.push_back(Face.mIndices[1]);
    mesh.indices.push_back(Face.mIndices[2]);
  }
}
////////////////////////////////////////////////////////////////////////////////

void SkeletalAnimationRessource::upload_to(RenderContext const& ctx) /*const*/{

  if (vertices_.size() <= ctx.id || vertices_[ctx.id] == nullptr) {

    mesh = Mesh{};
    InitMesh(mesh);

    std::unique_lock<std::mutex> lock(upload_mutex_);

    if (vertices_.size() <= ctx.id) {
      vertices_.resize(ctx.id + 1);
      indices_.resize(ctx.id + 1);
      vertex_array_.resize(ctx.id + 1);
    }

    vertices_[ctx.id] =
        ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                         scm::gl::USAGE_STATIC_DRAW,
                                         mesh_.num_vertices * sizeof(Vertex),
                                         0);


    Vertex* data(static_cast<Vertex*>(ctx.render_context->map_buffer(
        vertices_[ctx.id], scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));

    for (unsigned v(0); v < mesh.num_vertices; ++v) {

      data[v].pos = mesh.positions[v];

      data[v].tex = mesh.texCoords[v];

      data[v].normal = mesh.normals[v];

      data[v].tangent = mesh.tangents[v];

      data[v].bitangent = mesh.bitangents[v];

      data[v].bone_weights = scm::math::vec4f(mesh.bones[v].Weights[0],mesh.bones[v].Weights[1],mesh.bones[v].Weights[2],mesh.bones[v].Weights[3]);
      
      data[v].bone_ids = scm::math::vec4i(mesh.bones[v].IDs[0],mesh.bones[v].IDs[1],mesh.bones[v].IDs[2],mesh.bones[v].IDs[3]);
    }

    ctx.render_context->unmap_buffer(vertices_[ctx.id]);

    indices_[ctx.id] =
        ctx.render_device->create_buffer(scm::gl::BIND_INDEX_BUFFER,
                                         scm::gl::USAGE_STATIC_DRAW,
                                         mesh.num_triangles * 3 * sizeof(unsigned),
                                         &mesh.indices[0]);

    vertex_array_[ctx.id] = ctx.render_device->create_vertex_array(
        scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
            0, 1, scm::gl::TYPE_VEC2F, sizeof(Vertex))(
            0, 2, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
            0, 3, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
            0, 4, scm::gl::TYPE_VEC3F, sizeof(Vertex))(
            0, 5, scm::gl::TYPE_VEC4F, sizeof(Vertex))(
            0, 6, scm::gl::TYPE_VEC4I, sizeof(Vertex)),
        {vertices_[ctx.id]});
    
    // init non transformated/animated bone boxes
    // use every single vertex to be manipulated by a certain bone per bone box
    for (unsigned v(0); v < mesh.num_vertices; ++v) {
      auto final_pos  = scm::math::vec4(mesh.positions[v].x, mesh.positions[v].y, mesh.positions[v].z, 1.0);
      for(unsigned i(0); i<4; ++i){
        std::cout << mesh.bones[v].IDs[i] << std::endl;
        bone_boxes_[mesh.bones[v].IDs[i]].expandBy(scm::math::vec3(final_pos.x,final_pos.y,final_pos.z));
      }
    }

    ctx.render_context->apply();
  }

}

////////////////////////////////////////////////////////////////////////////////

std::vector<math::BoundingBox<math::vec3>>
SkeletalAnimationRessource::get_bone_boxes(){
  
  auto tmp_boxes = std::vector<math::BoundingBox<math::vec3>>(100,math::BoundingBox<math::vec3>());

  auto bone_transformation = animation_director_->get_bone_transforms();

  for(uint b(0);b<bone_boxes_.size();++b){

    if(!bone_boxes_[b].isEmpty()){
      tmp_boxes[b] = transform(bone_boxes_[b], bone_transformation[b]);
    }
  }
  return tmp_boxes;
}


////////////////////////////////////////////////////////////////////////////////

void SkeletalAnimationRessource::draw(RenderContext const& ctx) /*const*/ {

  // upload to GPU if neccessary
  upload_to(ctx);

  ctx.render_context->bind_vertex_array(vertex_array_[ctx.id]);
  ctx.render_context->bind_index_buffer(indices_[ctx.id], scm::gl::PRIMITIVE_TRIANGLE_LIST, scm::gl::TYPE_UINT);
  ctx.render_context->apply_vertex_input();
  ctx.render_context->draw_elements(mesh_.num_triangles * 3);
}

////////////////////////////////////////////////////////////////////////////////

void SkeletalAnimationRessource::ray_test(Ray const& ray, int options,
                    node::Node* owner, std::set<PickResult>& hits) {
  //TODO raycasting
  Logger::LOG_ERROR << "get_vertex() dynamic ray testing not supported " << std::endl;
  //kd_tree_.ray_test(ray, mesh_, options, owner, hits);
}

////////////////////////////////////////////////////////////////////////////////

unsigned int SkeletalAnimationRessource::num_vertices() const { return mesh_.num_vertices; }

////////////////////////////////////////////////////////////////////////////////

unsigned int SkeletalAnimationRessource::num_faces() const { return mesh_.num_triangles; }

////////////////////////////////////////////////////////////////////////////////

scm::math::vec3 SkeletalAnimationRessource::get_vertex(unsigned int i) const {

  //TODO physics handling
  Logger::LOG_ERROR << "get_vertex() dynamic vertex positions not supported " << std::endl;
  return scm::math::vec3();
}

////////////////////////////////////////////////////////////////////////////////

std::vector<unsigned int> SkeletalAnimationRessource::get_face(unsigned int i) const {

  //TODO cpu representation of mesh
  Logger::LOG_ERROR << "get_face() of merged neshes not supported " << std::endl;
  /*std::vector<unsigned int> face(mesh_->mFaces[i].mNumIndices);
  for (unsigned int j = 0; j < mesh_->mFaces[i].mNumIndices; ++j)
    face[j] = mesh_->mFaces[i].mIndices[j];
  return face;*/
  return std::vector<unsigned int>();
}

////////////////////////////////////////////////////////////////////////////////
}