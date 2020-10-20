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

/* This plugin is used to publish the encoders values of the Gazebo model of Robobo and to create the ResetWheels
service to reset wheel position values*/

#include "include/robobo/encoders.h"

namespace gazebo
{
// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(Encoders)

Encoders::Encoders()
{
  lReset = 0;
  rReset = 0;
}
Encoders::~Encoders()
{
}

void Encoders::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Safety check
  if (_model->GetJointCount() == 0)
  {
    std::cerr << "Invalid joint count, model not loaded\n";
    return;
  }

  model = _model;

  // Initialize ros
  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, model->GetName(), ros::init_options::NoSigintHandler);
  }

  std::string robot_namespace = "";
  if (_sdf->HasElement("robotNamespace")) {
    robot_namespace = _sdf->GetElement("robotNamespace")->Get<std::string>();
  }
  // Create node handler
  rosNode.reset(new ros::NodeHandle(robot_namespace));

  // Subscribe to Gazebo link states topic/
  connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&Encoders::Callback, this, _1));

  // Create topics to publish
  pubWheels = rosNode->advertise<robobo_msgs::Wheels>("wheels", 1);
  pubPan = rosNode->advertise<std_msgs::Int16>("pan", 1);
  pubTilt = rosNode->advertise<std_msgs::Int16>("tilt", 1);

  // Declare the node as a subscriber
  rosQueueThread = std::thread(std::bind(&Encoders::QueueThread, this));

  // Create ResetWheels service
  resetService = rosNode->advertiseService<robobo_msgs::ResetWheels::Request, robobo_msgs::ResetWheels::Response>("resetWheels", boost::bind(&Encoders::CallbackResetWheels, this, _1, _2));
  resetWheelsSub = rosNode->subscribe("resetWheelsCommand", 2, &Encoders::ResetWheelsTopicCallback, this);
}

void Encoders::Callback(const common::UpdateInfo &)
{
  robobo_msgs::Wheels msgWheels;
  std_msgs::Int16 msgPan;
  std_msgs::Int16 msgTilt;
  if (pubWheels.getNumSubscribers() > 0)
  {
    // Read and transform values to degrees of right wheel joint
    int newRightWheelPos = int(round(model->GetJoint("right_motor")->GetAngle(0).Degree()) - rReset);
    int newRightWheelVel = int(round(model->GetJoint("right_motor")->GetVelocity(0) * 180 / M_PI));

    // Read and transform values to degrees of left wheel joint
    int newLeftWheelPos = int(round(model->GetJoint("left_motor")->GetAngle(0).Degree()) - lReset);
    int newLeftWheelVel = int(round(model->GetJoint("left_motor")->GetVelocity(0) * 180 / M_PI));
    if (rightWheelPos != newRightWheelPos || rightWheelVel != newLeftWheelVel || leftWheelPos != newLeftWheelPos ||
        leftWheelVel != newLeftWheelVel)
    {
      rightWheelVel = newRightWheelVel;
      rightWheelPos = newRightWheelPos;
      leftWheelVel = newLeftWheelVel;
      leftWheelPos = newLeftWheelPos;
      // Save data in Wheels msg
      msgWheels.wheelPosR.data = rightWheelPos;
      msgWheels.wheelSpeedR.data = rightWheelVel;
      msgWheels.wheelPosL.data = leftWheelPos;
      msgWheels.wheelSpeedL.data = leftWheelVel;
      // Publish msg in topic
      pubWheels.publish(msgWheels);
    }
  }
  if (pubPan.getNumSubscribers() > 0)
  {
    int newPanPos = int(round(model->GetJoint("pan_motor")->GetAngle(0).Degree()));
    if (panPos != newPanPos)
    {
      panPos = newPanPos;
      msgPan.data = panPos;
      pubPan.publish(msgPan);
    }
  }
  if (pubTilt.getNumSubscribers() > 0)
  {
    int newTiltPos = int(round(model->GetJoint("tilt_motor")->GetAngle(0).Degree()));
    if (tiltPos != newTiltPos)
    {
      tiltPos = newTiltPos;
      msgTilt.data = tiltPos;
      pubTilt.publish(msgTilt);
    }
  }
}

bool Encoders::CallbackResetWheels(robobo_msgs::ResetWheels::Request &req, robobo_msgs::ResetWheels::Response &res)
{
  ResetWheels();
  return true;
}
void Encoders::ResetWheelsTopicCallback(const boost::shared_ptr<robobo_msgs::ResetWheelsCommand const> &req)
{
  ResetWheels();
}

void Encoders::ResetWheels()
{
  rReset = round(model->GetJoint("right_motor")->GetAngle(0).Degree());
  lReset = round(model->GetJoint("left_motor")->GetAngle(0).Degree());
}

void Encoders::QueueThread()
{
  static const double timeout = 0.01;
  while (rosNode->ok())
  {
    rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

}  // namespace gazebo
