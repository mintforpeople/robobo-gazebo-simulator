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

#ifndef EMOTION_H_
#define EMOTION_H_

#include <boost/regex.hpp>
#include <std_msgs/ColorRGBA.h>

#include <gazebo/gazebo.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/msgs/msgs.hh>

#include "ros/ros.h"
#include <thread>

#include "robobo_msgs/SetEmotionCommand.h"
#include "robobo_msgs/SetEmotion.h"

namespace gazebo
{
class EmotionRobobo : public VisualPlugin
{
public:

  /// \brief Constructor
  EmotionRobobo();

  /// \brief Destructor
  ~EmotionRobobo();

  /// \brief Load the plugin
  void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf);

  void colorCallback(std::string emotion);

protected:
  bool ServiceCallback(robobo_msgs::SetEmotion::Request &req, robobo_msgs::SetEmotion::Response &res);
  void TopicCallback(const boost::shared_ptr<robobo_msgs::SetEmotionCommand const> &req);

private:

  ros::Publisher pubLed;

  /// \brief Pointer to the visual
  rendering::VisualPtr visual;

  /// \brief A ROS Node
  ros::NodeHandle* rosNode;

  /// \brief A ROS Subscriber
  ros::Subscriber rosSub;

  /// \brief A ROS service server
  ros::ServiceServer rosServ;

  /// \brief BlockId publisher
  ros::Publisher rosPub;

  /// \brief A thread the keeps running velocity setting
  std::thread emotionlsThread;

};
}  // namespace gazebo
#endif  // LED_COLOR_H
