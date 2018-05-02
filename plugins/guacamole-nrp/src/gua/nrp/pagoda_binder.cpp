#include <gua/nrp/nrp_cam_node.hpp>
#include <gua/nrp/nrp_node.hpp>
#include <gua/nrp/pagoda_binder.hpp>

namespace gua
{
namespace nrp
{
PagodaBinder::PagodaBinder()
    : _worker_mutex(), _scene(),

#if GUA_DEBUG == 1
      _log("transport", PagodaLog::LOG_LEVEL::DEBUG, true)
#else
      _log("transport", PagodaLog::LOG_LEVEL::ERROR, false)
#endif

{
    _worker_should_stop.store(false);
    _scene_initialized.store(false);
    _scene_frame_distance.store(0);
}
PagodaBinder::~PagodaBinder()
{
    _halt_transport_layer();
    _worker.join();
}
void PagodaBinder::bind_root_node(gua::nrp::NRPNode *root_node) { _scene.set_root_node(root_node); }
void PagodaBinder::bind_cam_node(gua::nrp::NRPCameraNode *cam_node) { _scene.set_cam_node(cam_node); }
void PagodaBinder::bind_transport_layer()
{
    _worker = std::thread([&] { _connect_to_transport_layer(0, nullptr); });
}
void PagodaBinder::pre_render() { _scene.pre_render(); }

void PagodaBinder::_connect_to_transport_layer(int argc, char **argv)
{
#if GUA_DEBUG == 1
    PagodaLog log("worker", PagodaLog::LOG_LEVEL::DEBUG, true);
#else
    PagodaLog log("worker", PagodaLog::LOG_LEVEL::ERROR, false);
#endif

    log.d("setup");

    gazebo::common::load();

    sdf::setFindCallback(boost::bind(&gazebo::common::find_file, _1));

    if(!gazebo::transport::init("invincible", 11345, 1))
    {
        log.e("Unable to initialize transport");
        throw std::runtime_error("Unable to initialize transport");
    }

    log.d("starting the model database, fetching models immediately");

    gazebo::common::ModelDatabase::Instance()->Start(true);

    gazebo::transport::run();

    log.d("init transport node");

    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    log.d("begin subscription");

    //    gazebo::transport::SubscriberPtr sub_scene = node->Subscribe("/gazebo/default/scene", &PagodaBinder::callback_scene, this);
    //
    //    gazebo::transport::SubscriberPtr sub_world = node->Subscribe("/gazebo/default/world_stats", &PagodaBinder::callback_world, this);
    //    gazebo::transport::SubscriberPtr sub_model = node->Subscribe("/gazebo/default/model/info", &PagodaBinder::callback_model_info, this);
    //    gazebo::transport::SubscriberPtr sub_pose_info = node->Subscribe("/gazebo/default/pose/info", &PagodaBinder::callback_pose_info, this);
    //    gazebo::transport::SubscriberPtr sub_material = node->Subscribe("/gazebo/default/material", &PagodaBinder::callback_material, this);
    //
    //    gazebo::transport::SubscriberPtr sub_factory_light = node->Subscribe("/gazebo/default/factory/light", &PagodaBinder::callback_factory_light, this);
    //    gazebo::transport::SubscriberPtr sub_modify_light = node->Subscribe("/gazebo/default/light/modify", &PagodaBinder::callback_modify_light, this);
    //
    //    gazebo::transport::SubscriberPtr sub_skeleton_pose_info = node->Subscribe("/gazebo/default/skeleton_pose/info", &PagodaBinder::callback_skeleton_pose_info, this);

    gazebo::transport::PublisherPtr pub_request = node->Advertise<gazebo::msgs::Request>("/gazebo/default/request", 60, 60);
    gazebo::transport::SubscriberPtr sub_response = node->Subscribe("/gazebo/default/response", &PagodaBinder::callback_response, this);

    log.d("subscription done");

    if(pub_request->WaitForConnection(gazebo::common::Time(5, 0)))
    {
        log.d("connection established");

        std::unique_lock<std::mutex> lk(_worker_mutex);
        while(!_worker_cv.wait_for(lk, std::chrono::milliseconds(16), [&] { return _worker_should_stop.load(); }))
        {
            int_fast32_t scene_frame_distance = _scene_frame_distance.load();

            if(!_scene_initialized.load() || scene_frame_distance > 16)
            {
                auto request_scene = gazebo::msgs::CreateRequest("scene_info");
                pub_request->Publish(*(request_scene), true);

                _scene_frame_distance.store(0);
            }
            else
            {
                auto request_entity = gazebo::msgs::CreateRequest("entity_list");
                pub_request->Publish(*(request_entity), true);

                _scene_frame_distance.store(scene_frame_distance + 1);
            }
        }

        pub_request.reset();
        sub_response.reset();
    }
    else
    {
        log.e("connection not established");

        throw std::runtime_error("connection not established");
    }

    //    sub_scene.reset();
    //
    //    sub_world.reset();
    //    sub_model.reset();
    //    sub_pose_info.reset();
    //    sub_material.reset();
    //
    //    sub_factory_light.reset();
    //    sub_modify_light.reset();
    //
    //    sub_skeleton_pose_info.reset();

    node->Fini();
    node.reset();

    gazebo::transport::stop();
    gazebo::transport::fini();

    gazebo::common::ModelDatabase::Instance()->Fini();

    log.d("over");
}
void PagodaBinder::_halt_transport_layer()
{
    _worker_should_stop.store(true);
    _worker_cv.notify_all();
}
void PagodaBinder::callback_response(ConstResponsePtr &ptr)
{
#if GUA_DEBUG == 1
    auto start = std::chrono::high_resolution_clock::now();
#endif

    _log.d("callback_response");
    _log.d(ptr->request().c_str());

    if(ptr->request() == "scene_info")
    {
        gazebo::msgs::Scene scene_msg;

        if(scene_msg.ParseFromString(ptr->serialized_data()))
        {
            _log.d(scene_msg.DebugString().c_str());
            _scene.on_scene_msg(boost::make_shared<const gazebo::msgs::Scene>(scene_msg));

            _scene_initialized.store(true);
        }
    }
    else if(ptr->request() == "entity_list")
    {
        gazebo::msgs::Model_V models_msg;

        if(models_msg.ParseFromString(ptr->serialized_data()))
        {
            _log.d(models_msg.DebugString().c_str());
            for(uint32_t i = 0; i < models_msg.models().size(); i++)
            {
                _scene.on_model_msg(boost::make_shared<const gazebo::msgs::Model>(models_msg.models(i)));
            }
        }
    }

#if GUA_DEBUG == 1
    auto end = std::chrono::high_resolution_clock::now();
    float callback_response = std::chrono::duration<float, std::milli>(end - start).count();

    std::cout << "callback_response: " << callback_response << std::endl;
#endif
}
void PagodaBinder::callback_world(ConstWorldStatisticsPtr &_msg)
{
    _log.d("callback_world");
    _log.d(_msg->DebugString().c_str());
}
void PagodaBinder::callback_material(ConstMaterialPtr &_msg)
{
    _log.d("callback_material");
    _log.d(_msg->DebugString().c_str());
}
void PagodaBinder::callback_skeleton_pose_info(ConstPoseAnimationPtr &ptr)
{
    _log.d("callback_skeleton_pose_info");
    _log.d(ptr->DebugString().c_str());

    _scene.on_skeleton_pose_msg(ptr);
}
void PagodaBinder::callback_model_info(ConstModelPtr &ptr)
{
    _log.d("callback_model_info");
    _log.d(ptr->DebugString().c_str());

    _scene.on_model_msg(ptr);
}
void PagodaBinder::callback_pose_info(ConstPosesStampedPtr &ptr)
{
    _log.d("callback_pose_info");
    _log.d(ptr->DebugString().c_str());

    _scene.on_pose_msg(ptr);
}
void PagodaBinder::callback_scene(ConstScenePtr &ptr)
{
    _log.d("callback_scene");
    _log.d(ptr->DebugString().c_str());

    _scene.on_scene_msg(ptr);
}
void PagodaBinder::callback_factory_light(ConstLightPtr &ptr)
{
    _log.d("callback_factory_light");
    _log.d(ptr->DebugString().c_str());

    _scene.on_light_factory_msg(ptr);
}
void PagodaBinder::callback_modify_light(ConstLightPtr &ptr)
{
    _log.d("callback_modify_light");
    _log.d(ptr->DebugString().c_str());

    _scene.on_light_modify_msg(ptr);
}
std::mutex &PagodaBinder::get_scene_mutex() { return _scene.get_mutex_scenegraph(); }
}
}