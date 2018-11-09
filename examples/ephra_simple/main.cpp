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

// following lines can be used to get the type of a auto object
#include <typeinfo>
#include <boost/core/demangle.hpp>
template<typename T>
std::string type_str(){ return boost::core::demangle(typeid(T).name()); }
// std::cout << "typeof(i) = " << type_str<decltype(AUTO_INSTANCE)>() << '\n';


int main(int argc, char** argv) {
  // initialize guacamole
  gua::init(argc, argv);

  // setup scene
  gua::SceneGraph graph("main_scenegraph");

  gua::DynamicTriangleLoader line_strip_loader;
  // gua::DynamicGeometryLoader line_strip_loader;
  // gua::LineStripLoader line_strip_loader;

  auto line_strip_example_node(line_strip_loader
                              .create_empty_geometry("ls_example_node", 
                                                     "empty_node.lob"));

  // get type of example node
  std::cout << "typeof(i) = " << type_str<decltype(line_strip_example_node)>() << '\n';

  auto transform = graph.add_node<gua::node::TransformNode>("/", "transform");
  
  // auto node = std::dynamic_pointer_cast<gua::node::DynamicGeometryNode>(line_strip_example_node);
  auto node = std::dynamic_pointer_cast<gua::node::DynamicTriangleNode>(line_strip_example_node);
  // auto node = std::dynamic_pointer_cast<gua::node::LineStripNode>(line_strip_example_node);

  node->push_vertex(-1.0f,  1.0f, -4.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.1f, 0.0f, 1.0f);
  node->push_vertex( 1.0f, -1.0f, -4.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.1f, 0.0f, 5.0f);
  node->push_vertex( 1.0f,  1.0f, -4.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f, 1.0f);

  // node->push_vertex(-1.0f, 1.0f, -4.0,0.0f,0.0f,0.0f,1.0f, 0.1f);
  // node->push_vertex(-1.0f, -1.0f, -4.0,0.0f,0.0f,0.0f,1.0f, 0.1f);
  // node->push_vertex(1.0f, -1.0f, -4.0,0.0f,0.0f,0.0f,1.0f, 0.1f);


  node->set_draw_bounding_box(true);
  node->set_render_volumetric(false);
  //node->set_screen_space_line_width(2.5f);
   
  graph.add_node("/transform", line_strip_example_node);

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
  

  gua::Renderer renderer;

  // application loop
  gua::events::MainLoop loop;
  gua::events::Ticker ticker(loop, 1.0 / 500.0);

  scm::math::vec3d camera_positon(0.0, 0.0, 2.0);
  scm::math::vec3d negative_z_viewing_direction(0.0, 0.0, -100.0);
  scm::math::vec3d::value_type t_max(200.0);

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

      if(0 == (++frame_count) % 100) {
        
      }
      
      renderer.queue_draw({&graph});
    }
  });

  loop.start();

  return 0;
}
