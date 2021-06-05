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

#include "include/robobo/emotion.h"

namespace gazebo
{
// Register the plugin in Gazebo.
  GZ_REGISTER_VISUAL_PLUGIN(EmotionRobobo)

// Constructor
EmotionRobobo::EmotionRobobo()
{
}
// Destructor
EmotionRobobo::~EmotionRobobo()
{
  delete this->rosNode;
}

void EmotionRobobo::Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf){
  this->visual = _parent;

  if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
    }

  std::string robot_namespace = "";
  if (_sdf->HasElement("robotNamespace")) {
    robot_namespace = _sdf->GetElement("robotNamespace")->Get<std::string>();
  }

  // Create node handler
  this->rosNode = new ros::NodeHandle(robot_namespace);
  pubLed = rosNode->advertise<robobo_msgs::SetEmotionCommand>("emotion", 1);

  // Create Led service
  rosServ = rosNode->advertiseService<robobo_msgs::SetEmotion::Request, robobo_msgs::SetEmotion::Response>("setEmotion", boost::bind(&EmotionRobobo::ServiceCallback, this, _1, _2));

  // Create Led Subscriber
  rosSub = rosNode->subscribe<robobo_msgs::SetEmotionCommand>("emotion", 10,&EmotionRobobo::TopicCallback, this);
}

bool EmotionRobobo::ServiceCallback(robobo_msgs::SetEmotion::Request &req, robobo_msgs::SetEmotion::Response &res)
{
  robobo_msgs::SetEmotionCommand resPub;
  std::string emotion= req.emotion.data;
  if (emotion=="happy" || emotion=="laughing" || emotion=="sad" || emotion=="angry" || emotion=="surprised" || emotion=="normal"){
    resPub.emotion.data=emotion;
    pubLed.publish(resPub);
  }
  else {return false;}
  return true;
}


void EmotionRobobo::TopicCallback(const boost::shared_ptr<robobo_msgs::SetEmotionCommand const> &req)
{
  std::string emotion= req->emotion.data;
  if (emotion=="happy" || emotion=="laughing" || emotion=="sad" || emotion=="angry" || emotion=="surprised" || emotion=="normal"){
    emotionlsThread = std::thread(std::bind(&EmotionRobobo::colorCallback, this, req->emotion.data));
    emotionlsThread.detach();
  }
}

void EmotionRobobo::colorCallback(std::string emotion){

  if (emotion=="happy"){
      this->visual->SetMaterial("emotion/smile",true,false);
  }
  if (emotion=="laughing"){
      this->visual->SetMaterial("emotion/laughing",true,false);
  }
  if (emotion=="sad"){
      this->visual->SetMaterial("emotion/sad",true,false);
  }
  if (emotion=="angry"){
      this->visual->SetMaterial("emotion/angry",true,false);
  }
  if (emotion=="normal"){
      this->visual->SetMaterial("emotion/normal",true,false);
  }
  if (emotion=="laughing"){
      this->visual->SetMaterial("emotion/laughing",true,false);
  }
}
}  // namespace gazebo
