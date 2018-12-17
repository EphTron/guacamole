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

#include <functional>

#include <gua/guacamole.hpp>
#include <gua/renderer/TriMeshLoader.hpp>
#include <gua/renderer/ToneMappingPass.hpp>
#include <gua/renderer/DebugViewPass.hpp>
#include <gua/utils/Trackball.hpp>

#include <gua/renderer/DynamicTrianglePass.hpp>
#include <gua/virtual_texturing/DeferredVirtualTexturingPass.hpp>

// following lines can be used to get the type of a auto object
#include <typeinfo>
#include <boost/core/demangle.hpp>
template<typename T>
std::string type_str(){ return boost::core::demangle(typeid(T).name()); }
// std::cout << "typeof(i) = " << type_str<decltype(AUTO_INSTANCE)>() << '\n';

// forward mouse interaction to trackball
void mouse_button(gua::utils::Trackball& trackball,
                  int mousebutton,
                  int action,
                  int mods) {
  gua::utils::Trackball::button_type button;
  gua::utils::Trackball::state_type state;

  switch (mousebutton) {
    case 0:
      button = gua::utils::Trackball::left;
      break;
    case 2:
      button = gua::utils::Trackball::middle;
      break;
    case 1:
      button = gua::utils::Trackball::right;
      break;
  };

  switch (action) {
    case 0:
      state = gua::utils::Trackball::released;
      break;
    case 1:
      state = gua::utils::Trackball::pressed;
      break;
  };

  trackball.mouse(button, state, trackball.posx(), trackball.posy());
}


int main(int argc, char** argv) {
  // initialize guacamole
  gua::init(argc, argv);

  // setup scene
  gua::SceneGraph graph("main_scenegraph");

  // VT STEP 1/5: - create a material
  auto earth_vt_mat = gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->make_new_material();
  earth_vt_mat->set_uniform("earth_vt_mat", std::string("/home/ephtron/Projects/avango/examples/ephras_app/data/salem.atlas"));
  

  // VT STEP 2/5: - load *.atlas-File as uniform
  // earth_vt_mat->set_uniform("earth_vt_mat", std::string("/opt/3d_models/virtual_texturing/earth_colour_86400x43200_256x256_1_rgb.atlas"));
  // earth_vt_mat->set_uniform("earth_vt_mat", std::string("/home/ephtron/e_vive_controller/onepointfive_texture_2048_w2048_h2048.atlas"));

  // VT STEP 3/5: - enable virtual texturing for this material
  earth_vt_mat->set_enable_virtual_texturing(true);

  // // make texture 
  // gua::TextureDatabase::instance()->load("data/objects/bottle/albedo.png");

  // mat_bottle->set_uniform("ColorMap",     std::string("data/objects/bottle/albedo.png"))
  // gua::TextureDatabase::instance()->load("data/textures/out.png");
  // auto my_texture_mat = gua::MaterialShaderDatabase::instance()->lookup("gua_default_material")->make_new_material();
  // my_texture_mat->set_uniform("ColorMap", std::string("data/textures/out.png"))
  //                .set_show_back_faces(true);  ///// USE IF NO VT

  gua::DynamicTriangleLoader line_strip_loader;

  auto line_strip_example_node(line_strip_loader
                              .create_empty_geometry("ls_example_node", 
                                                     "empty_node.lob",
                                                      earth_vt_mat,
                                                       // my_texture_mat, // use iF NOT VT
                                                      gua::TriMeshLoader::NORMALIZE_POSITION |
                                                      gua::TriMeshLoader::NORMALIZE_SCALE |  
                                                      gua::TriMeshLoader::MAKE_PICKABLE));

  // get type of example node
  // std::cout << "typeof(i) = " << type_str<decltype(line_strip_example_node)>() << '\n';

  auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");
  
  // auto node = std::dynamic_pointer_cast<gua::node::DynamicGeometryNode>(line_strip_example_node);
  auto node = std::dynamic_pointer_cast<gua::node::DynamicTriangleNode>(line_strip_example_node);
  // auto node = std::dynamic_pointer_cast<gua::node::LineStripNode>(line_strip_example_node);
  //for (int i = 0; i < 200; ++i){  // TEST THE PERFMANCE
    node->push_vertex(-1.0f,  1.0f, -4.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.1f, 0.1f, 0.2f);
    node->push_vertex( 1.0f, -1.0f, -4.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.1f, 0.2f, 0.1f);
    node->push_vertex( 1.0f,  1.0f, -4.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.2f, 0.20f);

    // node->push_vertex(-1.0f,  1.0f, -4.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.1f, 0.0f, 0.0f);
    // node->push_vertex(-1.0f, -1.0f, -4.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.1f, 0.0f, 1.0f);
    // node->push_vertex( 1.0f, -1.0f, -4.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 1.0f, 0.0f);
  //}



  node->set_draw_bounding_box(true);
  node->set_render_volumetric(false);
  //node->set_screen_space_line_width(2.5f);
   
  graph.add_node("/transform", line_strip_example_node);

  gua::TriMeshLoader loader;

  auto ray_geometry(loader.create_geometry_from_file(
      "ray_geometry", "data/objects/cylinder.obj",
      gua::TriMeshLoader::NORMALIZE_POSITION |
      gua::TriMeshLoader::NORMALIZE_SCALE)  );
  graph.add_node("/", ray_geometry);

  ray_geometry->scale(0.02, 0.02, 0.1);

  // auto fking_plane(loader.create_geometry_from_file(
  //     "fking_plane", "data/objects/plane.obj",
  //     my_texture_mat,
  //     gua::TriMeshLoader::NORMALIZE_POSITION |
  //     gua::TriMeshLoader::NORMALIZE_SCALE)  );
  // graph.add_node("/transform", fking_plane);

  // fking_plane->rotate(90, 1.f, 0.f, 0.f);
  // fking_plane->rotate(180, 0.f, 0.f, 1.f);

  scm::math::vec3d cube_translation(0.0, 0.0, -5.0);

  auto light2 = graph.add_node<gua::node::LightNode>("/", "light2");
  light2->data.set_type(gua::node::LightNode::Type::POINT);
  light2->data.brightness = 150.0f;
  light2->scale(12.f);
  light2->translate(-3.f, 5.f, 5.f);

  auto screen = graph.add_node<gua::node::ScreenNode>("/", "screen");
  screen->data.set_size(gua::math::vec2(1.92f, 1.08f));
  screen->translate(0, 0, 1.0);


  // add mouse interaction
  gua::utils::Trackball trackball(0.01, 0.002, 0.2);

  // setup rendering pipeline and window
  auto resolution = gua::math::vec2ui(1920, 1080);

  auto camera = graph.add_node<gua::node::CameraNode>("/screen", "cam");
  camera->translate(0, 0, 2.0);
  camera->config.set_resolution(resolution);
  camera->config.set_screen_path("/screen");
  camera->config.set_scene_graph_name("main_scenegraph");
  camera->config.set_output_window_name("Ephra Simple Example");
  camera->config.set_enable_stereo(false);

  camera->get_pipeline_description()->get_resolve_pass()->tone_mapping_exposure(
    1.0f);

  auto pipe = std::make_shared<gua::PipelineDescription>();
  pipe->add_pass(std::make_shared<gua::TriMeshPassDescription>());
  pipe->add_pass(std::make_shared<gua::DynamicTrianglePassDescription>());
  // VT STEP 5/5: - add DeferredVirtualTexturingPassDescription

  pipe->add_pass(std::make_shared<gua::DeferredVirtualTexturingPassDescription>()); // <- ONLY USE THIS PASS IF YOU LOAD VT MODELS
  pipe->add_pass(std::make_shared<gua::LightVisibilityPassDescription>());
  
  auto resolve_pass = std::make_shared<gua::ResolvePassDescription>();
  resolve_pass->background_mode(
      gua::ResolvePassDescription::BackgroundMode::QUAD_TEXTURE);
  resolve_pass->tone_mapping_exposure(1.0f);
  pipe->add_pass(resolve_pass);
  camera->set_pipeline_description(pipe);

  auto window = std::make_shared<gua::GlfwWindow>();
  gua::WindowDatabase::instance()->add("Ephra Simple Example", window);

  window->config.set_enable_vsync(false);
  window->config.set_size(resolution);
  window->config.set_resolution(resolution);
  window->config.set_stereo_mode(gua::StereoMode::MONO);

  window->on_resize.connect([&](gua::math::vec2ui const& new_size) {
    window->config.set_resolution(new_size);
    camera->config.set_resolution(new_size);
    screen->data.set_size(
        gua::math::vec2(0.001 * new_size.x, 0.001 * new_size.y));
  });
  window->on_move_cursor.connect(
      [&](gua::math::vec2 const& pos) { trackball.motion(pos.x, pos.y); });
  window->on_button_press.connect(
      std::bind(mouse_button, std::ref(trackball), std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3));


  gua::Renderer renderer;

  // application loop
  gua::events::MainLoop loop;
  gua::events::Ticker ticker(loop, 1.0 / 500.0);

  scm::math::vec3d camera_positon(0.0, 0.0, 2.0);
  scm::math::vec3d negative_z_viewing_direction(0.0, 0.0, -100.0);
  scm::math::vec3d::value_type t_max(200.0);

  //create ray that is located in the origin of the current screen and looks along the negative z-axis (pick ray is seen as circle)
  gua::Ray ray_from_camera_position(camera_positon, negative_z_viewing_direction, t_max);


  size_t frame_count = 0;

  ticker.on_tick.connect([&]() {

    // apply trackball matrix to object
    gua::math::mat4 modelmatrix =
        scm::math::make_translation(cube_translation[0], cube_translation[1], cube_translation[2]) * 
        scm::math::make_translation(trackball.shiftx(), trackball.shifty(),
                                    trackball.distance()) *
        gua::math::mat4(trackball.rotation());

    transform->set_transform(modelmatrix);

    if (window->should_close()) {
      renderer.stop();
      window->close();
      loop.stop();
    } else {

      //node->update_vertex(0, -1.0f,  1.0f, -4.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.1f, (frame_count % 300) / 300.0f, 0.0f);
      auto pick_results = graph.ray_test(ray_from_camera_position, gua::PickResult::Options::PICK_ONLY_FIRST_FACE | gua::PickResult::Options::GET_WORLD_POSITIONS);

      if(0 == (++frame_count) % 100) {
          
          std::cout << "World space intersection position: " << pick_results.begin()->world_position << "\n";
          std::cout << "World NAME : " << pick_results.begin()->object << "\n";
      }
      
      renderer.queue_draw({&graph});
    }
  });

  loop.start();

  return 0;
}

