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

#include "include/robobo/led_color.h"

namespace gazebo
{
// Register the plugin in Gazebo.
  GZ_REGISTER_VISUAL_PLUGIN(LedColorRobobo)

// Constructor
LedColorRobobo::LedColorRobobo()
{
}
// Destructor
LedColorRobobo::~LedColorRobobo()
{
  delete this->rosNode;
}

void LedColorRobobo::Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf){
  this->visual = _parent->GetParent();

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
  pubLed = rosNode->advertise<robobo_msgs::Led>("leds", 1);

  // Create Led service
  rosServ = rosNode->advertiseService<robobo_msgs::SetLed::Request, robobo_msgs::SetLed::Response>("setLed", boost::bind(&LedColorRobobo::ServiceCallback, this, _1, _2));

  // Create Led Subscriber
  rosSub = rosNode->subscribe<robobo_msgs::Led>("leds", 10,&LedColorRobobo::TopicCallback, this);
}

bool LedColorRobobo::ServiceCallback(robobo_msgs::SetLed::Request &req, robobo_msgs::SetLed::Response &res)
{
  robobo_msgs::Led resPub;
  std::string id= req.id.data;

  if(req.color.data == "red"){resPub.color.r = 1.0; resPub.color.g = 0.0; resPub.color.b = 0.0;}
  else if(req.color.data == "blue"){resPub.color.r = 0.0; resPub.color.g = 0.0; resPub.color.b = 1.0;}
  else if(req.color.data == "cyan"){resPub.color.r = 0.0; resPub.color.g = 1.0; resPub.color.b = 1.0;}
  else if(req.color.data == "magenta"){resPub.color.r = 1.0; resPub.color.g = 0.0; resPub.color.b = 1.0;}
  else if(req.color.data == "yellow"){resPub.color.r = 1.0; resPub.color.g = 1.0; resPub.color.b = 0.0;}
  else if(req.color.data == "green"){resPub.color.r = 0.0; resPub.color.g = 1.0; resPub.color.b = 0.0;}
  else if(req.color.data == "orange"){resPub.color.r = 1.0; resPub.color.g = 0.5; resPub.color.b = 0.0;}
  else if(req.color.data == "off"){resPub.color.r = -1.0; resPub.color.g = 0.0; resPub.color.b = 0.0;}
  else{return false;}

  if (id=="Front-C" || id=="Front-R" || id=="Front-RR" || id=="Front-L" || id=="Front-LL" ||
      id=="Back-R" || id=="Back-L"){ resPub.id.data=id;}
  else if (id=="all"){
    resPub.id.data="Front-C"; pubLed.publish(resPub); usleep(5000);
    resPub.id.data="Front-R"; pubLed.publish(resPub); usleep(5000);
    resPub.id.data="Front-RR"; pubLed.publish(resPub); usleep(5000);
    resPub.id.data="Front-L"; pubLed.publish(resPub); usleep(5000);
    resPub.id.data="Front-LL"; pubLed.publish(resPub); usleep(5000);
    resPub.id.data="Back-R"; pubLed.publish(resPub); usleep(5000);
    id="Back-L";
  }
  else {return false;}
  resPub.id.data=id;
  pubLed.publish(resPub);
  return true;
}


void LedColorRobobo::TopicCallback(const boost::shared_ptr<robobo_msgs::Led const> &req)
{
  int id_led;

  if (req->id.data=="Front-C" || req->id.data=="Front-R" || req->id.data=="Front-RR" || 
  req->id.data=="Front-L" || req->id.data=="Front-LL" || req->id.data=="Back-R" || req->id.data=="Back-L"){
    boost::regex ledChildId(req->id.data+"_");
    for (int i=1;i<8;i++){
      if (regex_search(this->visual->GetChild(i)->Name(), ledChildId)){
        id_led=i;
      }
    }
  }
  else {id_led=10;}

  ledlsThread = std::thread(std::bind(&LedColorRobobo::colorCallback, this, id_led, req->color.r, req->color.g, req->color.b));
  ledlsThread.detach();
}

void LedColorRobobo::colorCallback(int id_led, float r, float g, float b){

  if (id_led!=10){
    if (r!=-1){
      ignition::math::Color newColor= ignition::math::Color(r,g,b,1.0);
      ignition::math::Color onColor= ignition::math::Color(0,0,0,1.0);
      this->visual->GetChild(id_led)->SetDiffuse(onColor);
      this->visual->GetChild(id_led)->SetAmbient(onColor);
      this->visual->GetChild(id_led)->SetEmissive(newColor);
    }
    else{
      ignition::math::Color newColor= ignition::math::Color(0.0,0.0,0.0,0.0);
      this->visual->GetChild(id_led)->SetDiffuse(newColor);
      this->visual->GetChild(id_led)->SetAmbient(newColor);
      this->visual->GetChild(id_led)->SetEmissive(newColor);
      
    }
  }
}
}  // namespace gazebo
