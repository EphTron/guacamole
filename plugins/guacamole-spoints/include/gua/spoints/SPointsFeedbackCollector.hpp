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

#ifndef GUA_SPOINTS_FEEDBACK_COLLECTOR_HPP
#define GUA_SPOINTS_FEEDBACK_COLLECTOR_HPP

// guacamole headers
#include <gua/spoints/platform.hpp>
#include <gua/databases/Database.hpp>
#include <zmq.hpp>

#include <gua/spoints/spoints_geometry/NetKinectArray.hpp>
// external headers
#include <string>
#include <list>
#include <memory>
#include <unordered_set>
#include <unordered_map>

namespace gua {


class GUA_SPOINTS_DLL SPointsFeedbackCollector : public Singleton<SPointsFeedbackCollector> {
 public:


  SPointsFeedbackCollector() {
    feedback_zmq_context_ = std::make_shared<zmq::context_t>(1); // means single threaded
  };
  ~SPointsFeedbackCollector() = default;


  void push_feedback_matrix(RenderContext const& ctx, std::string const& socket_string, spoints::matrix_package const& pushed_feedback_matrix) {

      std::unique_lock<std::mutex> lock(m_feedback_mutex_);

    //std::cout << "Context " << ctx.id << " pushed feedback\n";

      //++num_called_push_feedbacks_per_context_[ctx.id];

          
      auto last_seen_frame_count_iterator_for_context = last_seen_application_frame_count_per_context_.find(ctx.id);
      bool swap_feedback_for_context = false;
      
      /*if (last_seen_application_frame_count_per_context_.end() == last_seen_frame_count_iterator_for_context) {
        swap_feedback_for_context = true;
      } else {
        if(last_seen_application_frame_count_per_context_[ctx.id] != ctx.framecount) {
          swap_feedback_for_context = true;
        }
      }

      last_seen_application_frame_count_per_context_[ctx.id] = ctx.framecount;
      */

      /*if(swap_feedback_for_context) {
        std::swap(queued_feedback_packages_per_context_per_socket_[ctx.id], finalized_feedback_packages_per_context_per_socket_[ctx.id]);
        queued_feedback_packages_per_context_per_socket_[ctx.id].clear();


      }*/

        //queued_feedback_packages_per_context_per_socket_

        auto const& collected_feedback_matrices_for_socket = queued_feedback_packages_per_context_per_socket_[ctx.id][socket_string];
        for (auto const& curr_matrix_package : collected_feedback_matrices_for_socket) {
          if (!memcmp ( &curr_matrix_package, &pushed_feedback_matrix, sizeof(curr_matrix_package) ) ) {
            //std::cout << "BUT WAS ALREADY REGISTERED\n";
            return;
          }
        }
        queued_feedback_packages_per_context_per_socket_[ctx.id][socket_string].push_back(pushed_feedback_matrix);
      

  }


  void send_feedback_frame(RenderContext const& ctx, std::size_t application_frame_count) {



    std::unique_lock<std::mutex> lock(m_feedback_mutex_);

    std::swap(queued_feedback_packages_per_context_per_socket_[ctx.id], finalized_feedback_packages_per_context_per_socket_[ctx.id]);
    queued_feedback_packages_per_context_per_socket_[ctx.id].clear();

    std::cout << "Trying to send feedback frame\n";
    std::map<std::string, std::vector<spoints::matrix_package>> serialized_matrices_per_socket; 

    for(auto const& all_feedback_per_socket_per_context : finalized_feedback_packages_per_context_per_socket_) {
      std::vector<spoints::matrix_package> collected_finalized_matrices_for_socket;
      for (auto const& all_feedback_per_socket_for_current_context : all_feedback_per_socket_per_context.second) {
        std::string current_socket_string = all_feedback_per_socket_for_current_context.first;

        auto socket_iterator              = socket_per_socket_string_.find(current_socket_string);
        if (socket_per_socket_string_.end() == socket_iterator) {

          auto new_socket_ptr = std::make_shared<zmq::socket_t>(*feedback_zmq_context_, ZMQ_PUB);

          int conflate_messages  = 1;

          new_socket_ptr->setsockopt(ZMQ_CONFLATE, &conflate_messages, sizeof(conflate_messages));

          std::string endpoint(std::string("tcp://") + current_socket_string.c_str());

          try { 
            new_socket_ptr->bind(endpoint.c_str()); 
          } catch (const std::exception& e) {
            std::cout << "Failed to bind feedback socket\n";
            return;
          }

          socket_per_socket_string_.insert(std::make_pair(current_socket_string, new_socket_ptr) );
        }




        auto const& matrix_packages_to_submit = all_feedback_per_socket_for_current_context.second;

        auto& matrix_collection_vector_for_socket_string = serialized_matrices_per_socket[current_socket_string];
        matrix_collection_vector_for_socket_string.insert(matrix_collection_vector_for_socket_string.end(), 
          matrix_packages_to_submit.begin(), matrix_packages_to_submit.end());



 



        //for (auto const& recorded_matrix : matrix_packages_to_submit) {
          //std::cout << "Mat " << mat_counter << ":\n";
          //std::cout << .mat_package << "\n";
        //}
      }


      for( auto const& collected_feedback_pair_per_socket : serialized_matrices_per_socket) {

        size_t feedback_header_byte = 100;

        uint32_t num_recorded_matrix_packages = 0;


        auto& current_socket = socket_per_socket_string_[collected_feedback_pair_per_socket.first];

        //serialized_matrices_per_socket

        auto const& collected_matrices = collected_feedback_pair_per_socket.second;

        num_recorded_matrix_packages = collected_matrices.size();

        //HEADER DATA SO FAR:

        // 00000000 uint32_t num_matrices

      
        zmq::message_t zmqm(feedback_header_byte + num_recorded_matrix_packages * sizeof(spoints::matrix_package) );

        memcpy((char*)zmqm.data(), (char*)&(num_recorded_matrix_packages), sizeof(uint32_t));     
        memcpy( ((char*)zmqm.data()) + (feedback_header_byte), (char*)&(collected_matrices[0]), (num_recorded_matrix_packages) *  sizeof(spoints::matrix_package) );

        std::cout << "actually recorded matrices: " << num_recorded_matrix_packages << "\n";
        // send feedback
        current_socket->send(zmqm); // blocking
      }



    }
    //feedback_packages_per_socket_[frame_count_id_to_send].clear();
  }

  std::mutex                                                                   m_feedback_mutex_;

  std::map<size_t, 
    std::map<std::string, std::vector<spoints::matrix_package>>>               finalized_feedback_packages_per_context_per_socket_;

  std::unordered_map<std::string,std::shared_ptr<zmq::socket_t>>               socket_per_socket_string_;
  std::shared_ptr<zmq::context_t>                                              feedback_zmq_context_;

  std::unordered_map<unsigned, bool>                                           seen_feedback_ids_;
  std::unordered_map<unsigned, int>                                            num_called_push_feedbacks_per_context_;
  std::unordered_map<unsigned, int>                                            num_attempted_send_feedback_calls_per_context_;
  std::unordered_map<std::size_t, std::size_t>                                 last_seen_application_frame_count_per_context_; 

  std::map<size_t, 
  std::map<std::string, std::vector<spoints::matrix_package>>>                 queued_feedback_packages_per_context_per_socket_;

};

}

#endif  // GUA_SPOINTS_FEEDBACK_COLLECTOR_HPP