 /*This work has been funded by rosin.org (contract agreement 732287) 
 through EP project "Robobo AI"*/

/*******************************************************************************
 *
 *   Copyright 2019, Manufactura de Ingenios Tecnológicos S.L.
 *   <http://www.mintforpeople.com>
 *
 *   Redistribution, modification and use of this software are permitted under
 *   terms of the Apache 2.0 License.
 *
 *   This software is distributed in the hope that it will be useful,
 *   but WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND; without even the implied
 *   warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *   Apache 2.0 License for more details.
 *
 *   You should have received a copy of the Apache 2.0 License along with
 *   this software. If not, see <http://www.apache.org/licenses/>.
 *
 ******************************************************************************/

#ifndef MOVE_WHEELS_PLUGIN_H_
#define MOVE_WHEELS_PLUGIN_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <cstdint>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>

#include "robobo_msgs/MoveWheels.h"
#include "robobo_msgs/MoveWheelsCommand.h"

namespace gazebo
{
class MoveWheels : public ModelPlugin
{
public:
  physics::JointControllerPtr jointController;

  /// \brief Constructor
  MoveWheels();

  /// \brief Destructor
  ~MoveWheels();

  /// \brief Load the controller
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
  bool ServiceCallback(robobo_msgs::MoveWheels::Request &req, robobo_msgs::MoveWheels::Response &res);
  void TopicCallback(const boost::shared_ptr<robobo_msgs::MoveWheelsCommand const> &msg);
  void QueueThread();

private:

  void HandleMoveWheels(int8_t lwp, int8_t rwp, float time, int16_t blockid);

  /// \brief Pointer to the model.
  physics::ModelPtr model;

  /// \brief A node use for ROS transport
  std::unique_ptr<ros::NodeHandle> rosNode;

  /// \brief A ROS service server
  ros::ServiceServer moveWheelsService;

  /// \brief A ROS Subscriber
  ros::Subscriber moveWheelsSubscriber;

  /// \brief BlockId publisher
  ros::Publisher blockPub;

  /// \brief A ROS callbackqueue that helps process messages
  ros::CallbackQueue rosQueue;

  /// \brief A thread the keeps running the rosQueue
  std::thread rosQueueThread;

  /// \brief A thread the keeps running velocity setting
  std::thread moveWheelsThread;
};

}  // namespace gazebo

#endif
