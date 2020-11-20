/*******************************************************************************
 *
 *   Copyright 2020, Manufactura de Ingenios Tecnológicos S.L.
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

#ifndef ENCODERS_H
#define ENCODERS_H
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>

#include "gazebo_msgs/LinkStates.h"
#include "robobo_msgs/ResetWheels.h"
#include "robobo_msgs/ResetWheelsCommand.h"
#include "robobo_msgs/Wheels.h"
#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Int16.h"

namespace gazebo
{
class Encoders : public ModelPlugin
{
public:
  /// \brief Constructor
  Encoders();

  /// \brief Destructor
  ~Encoders();

  /// \brief Load the plugin
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

protected:
  void Callback(const common::UpdateInfo &);
  bool CallbackResetWheels(robobo_msgs::ResetWheels::Request &req, robobo_msgs::ResetWheels::Response &res);
  void ResetWheelsTopicCallback(const boost::shared_ptr<robobo_msgs::ResetWheelsCommand const> &req);
  void QueueThread();

private:
  int rReset, lReset, rightWheelPos, leftWheelPos, rightWheelVel, leftWheelVel, panPos, tiltPos;

  /// \brief Pointer to the update event connection.
  event::ConnectionPtr connection;

  inline void ResetWheels();

  /// \brief Pointer to the model
  physics::ModelPtr model;

  /// \brief pointer to ros node
  std::unique_ptr<ros::NodeHandle> rosNode;

  ros::Publisher pubWheels;
  ros::Publisher pubPan;
  ros::Publisher pubTilt;

  /// \brief A ROS callback queue that helps process messages
  ros::CallbackQueue rosQueue;

  /// \brief A thread the keeps running the rosQueue
  std::thread rosQueueThread;

  /// \brief A ROS service server
  ros::ServiceServer resetService;

  ros::Subscriber linkStatesSub;
  ros::Subscriber resetWheelsSub;
};
}  // namespace gazebo
#endif  // ENCODERS_H
