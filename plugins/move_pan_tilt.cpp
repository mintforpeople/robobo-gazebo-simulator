/*******************************************************************************
 *
 *   Copyright 2019, Manufactura de Ingenios Tecnológicos S.L.
 *   <http://www.mintforpeople.com>
 *
 *   Redistribution, modification and use of this software are permitted under
 *   terms of the Apache 2.0 License.paracaidas
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

// This plugin is used to create the MovePanTilt service to move pan and tilt motors in the Gazebo model of Robobo

#include "include/robobo/move_pan_tilt.h"

namespace gazebo
{
// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(MovePanTilt)

MovePanTilt::MovePanTilt()
{
}
MovePanTilt::~MovePanTilt()
{
}

void MovePanTilt::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Safety check
  if (_model->GetJointCount() == 0)
  {
    std::cerr << "Invalid joint count, model not loaded\n";
    return;
  }

  model = _model;

  // Set pan and tilt initial positions
  model->GetJoint("tilt_motor")->SetPosition(0, 70*M_PI/180);
  model->GetJoint("tilt_motor")->SetLowStop(0, 70*M_PI/180);
  model->GetJoint("tilt_motor")->SetHighStop(0, 70*M_PI/180);
  model->GetJoint("pan_motor")->SetPosition(0, M_PI);
  model->GetJoint("pan_motor")->SetLowStop(0, M_PI);
  model->GetJoint("pan_motor")->SetHighStop(0, M_PI);

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

  // Create MovePanTilt service
  ros::AdvertiseServiceOptions advSerOpt = ros::AdvertiseServiceOptions::create<robobo_msgs::MovePanTilt>(
      "movePanTilt", boost::bind(&MovePanTilt::ServiceCallback, this, _1, _2), ros::VoidPtr(), &this->rosQueue);
  movePanTiltService = rosNode->advertiseService(advSerOpt);

  ros::SubscribeOptions subOpt = ros::SubscribeOptions::create<robobo_msgs::MovePanTiltCommand>(
      "movePanTilt", 10, boost::bind(&MovePanTilt::TopicCallback, this, _1), ros::VoidPtr(), &this->rosQueue);
  movePanTiltSubscriber = rosNode->subscribe(subOpt);


  // BlockId Publisher
  blockPub = rosNode->advertise<std_msgs::Int16>("unlock/move", 10);

  // Queue thread
  this->rosQueueThread = std::thread(std::bind(&MovePanTilt::QueueThread, this));}

bool MovePanTilt::ServiceCallback(robobo_msgs::MovePanTilt::Request &req, robobo_msgs::MovePanTilt::Response &res)
{
  int16_t panPos = (req.panPos.data < 11) ? 11 : (343 < req.panPos.data) ? 343 : req.panPos.data,
          tiltPos = (req.tiltPos.data < 26) ? 26 : (109 < req.tiltPos.data) ? 109 : req.tiltPos.data;
  int8_t panSpeed = (req.panSpeed.data < 0) ? 0 : (100 < req.panSpeed.data) ? 100 : req.panSpeed.data,
         tiltSpeed = (req.tiltSpeed.data < 0) ? 0 : (100 < req.tiltSpeed.data) ? 100 : req.tiltSpeed.data;

  movePanTiltThread = std::thread(std::bind(&MovePanTilt::HandleMovePanTilt, this, panPos, panSpeed,
                                            req.panUnlockId.data, tiltPos, tiltSpeed, req.tiltUnlockId.data));
  movePanTiltThread.join();
  return true;
}

void MovePanTilt::TopicCallback(const boost::shared_ptr<robobo_msgs::MovePanTiltCommand const> &req)
{
  int16_t panPos = (req->panPos.data < 11) ? 11 : (343 < req->panPos.data) ? 343 : req->panPos.data,
          tiltPos = (req->tiltPos.data < 26) ? 26 : (109 < req->tiltPos.data) ? 109 : req->tiltPos.data;
  int8_t panSpeed = (req->panSpeed.data < 0) ? 0 : (100 < req->panSpeed.data) ? 100 : req->panSpeed.data,
         tiltSpeed = (req->tiltSpeed.data < 0) ? 0 : (100 < req->tiltSpeed.data) ? 100 : req->tiltSpeed.data;

  movePanTiltThread = std::thread(std::bind(&MovePanTilt::HandleMovePanTilt, this, panPos, panSpeed,
                                            req->panUnlockId.data, tiltPos, tiltSpeed, req->tiltUnlockId.data));
  movePanTiltThread.join();
}

void MovePanTilt::HandleMovePanTilt(int16_t panPos, int8_t panSpeed, int16_t panBlockId, int16_t tiltPos,
                                    int8_t tiltSpeed, int16_t tiltBlockId)
{
  double motorVelocity;
  double currentPanAngle = model->GetJoint("pan_motor")->GetAngle(0).Degree();
  double currentTiltAngle = model->GetJoint("tilt_motor")->GetAngle(0).Degree();

  // Use no more than 0.15 Nm to achieve pan target velocity
  model->GetJoint("pan_motor")->SetParam("fmax", 0, 0.15);
  // Use no more than 0.22 Nm to achieve tilt target velocity
  model->GetJoint("tilt_motor")->SetParam("fmax", 0, 0.22);

  // Set pan position
  if (10 < panPos && panPos < 345 && panPos != round(currentPanAngle) && panBlockId != 0)
  {
    // Calculate joint target velocity
    int difPan = panPos - currentPanAngle;
    motorVelocity =
        (abs(difPan) * (-3.06E-05 * pow(panSpeed, 3) + 3.877E-03 * pow(panSpeed, 2) + 0.84747 * panSpeed + 8.05468) /
         (abs(difPan) -
          (-1.99E-05 * pow(panSpeed, 3) + 1.064E-03 * pow(panSpeed, 2) - 0.33034 * panSpeed - 8.9074E-01))) *
        M_PI / 180;

    // Unlock joint position and set target velocity
    model->GetJoint("pan_motor")->SetLowStop(0, 0.191);
    model->GetJoint("pan_motor")->SetHighStop(0, 6.004);
    // TODO: Busy waiting could be prevented with events on Gazebo 8+
    if (currentPanAngle < panPos)
    {
      while (model->GetJoint("pan_motor")->GetAngle(0).Degree() < panPos)
      {
        model->GetJoint("pan_motor")->SetParam("vel", 0, motorVelocity);
      }
    }
    else
    {
      while (model->GetJoint("pan_motor")->GetAngle(0).Degree() > panPos)
      {
        model->GetJoint("pan_motor")->SetParam("vel", 0, -motorVelocity);
      }
    }
    // Lock joint position
    model->GetJoint("pan_motor")->SetParam("vel", 0, 0);
    model->GetJoint("pan_motor")->SetLowStop(0, model->GetJoint("pan_motor")->GetAngle(0));
    model->GetJoint("pan_motor")->SetHighStop(0, model->GetJoint("pan_motor")->GetAngle(0));
    // TODO:check if force should be set to 0 and check if blockid should be published anyways
    if (blockPub.getNumSubscribers() > 0)
    {
      std_msgs::Int16 unblockId;
      unblockId.data = panBlockId;
      blockPub.publish(unblockId);
      ros::spinOnce();
    }
  }

  // Set tilt position
  if (4 < tiltPos && tiltPos < 111 && tiltPos != round(currentTiltAngle) && tiltBlockId != 0)
  {
    // Calculate joint target velocity
    int diftilt = tiltPos - currentTiltAngle;
    motorVelocity =
        (abs(diftilt) * (1.409E-05 * pow(tiltSpeed, 3) - 2.598E-03 * pow(tiltSpeed, 2) + 0.4809 * tiltSpeed + 3.182) /
         (abs(diftilt) -
          (-8.84E-05 * pow(tiltSpeed, 3) + 1.233E-02 * pow(tiltSpeed, 2) + -0.5495 * tiltSpeed + 4.738))) *
        M_PI / 180;

    // Unlock joint position and set target velocity
    model->GetJoint("tilt_motor")->SetLowStop(0, 0.0873);
    model->GetJoint("tilt_motor")->SetHighStop(0, 1.9199);
    // TODO: This busy wait could be prevented from Gazebo 8+
    if (currentTiltAngle < tiltPos)
    {
      while (model->GetJoint("tilt_motor")->GetAngle(0).Degree() < tiltPos)
      {
        model->GetJoint("tilt_motor")->SetParam("vel", 0, motorVelocity);
      }
    }
    else
    {
      while (model->GetJoint("tilt_motor")->GetAngle(0).Degree() > tiltPos)
      {
        // Unlock joint position and set target velocity
        model->GetJoint("tilt_motor")->SetParam("vel", 0, -motorVelocity);
      }
    }
    // Lock joint position
    model->GetJoint("tilt_motor")->SetParam("vel", 0, 0);
    model->GetJoint("tilt_motor")->SetLowStop(0, model->GetJoint("tilt_motor")->GetAngle(0));
    model->GetJoint("tilt_motor")->SetHighStop(0, model->GetJoint("tilt_motor")->GetAngle(0));
    // TODO: check if force should (or not) be set to 0 check if block id shold be published even if not moved
    if (blockPub.getNumSubscribers() > 0)
    {
      std_msgs::Int16 unblockId;
      unblockId.data = tiltBlockId;
      blockPub.publish(unblockId);
      ros::spinOnce();
    }
  }
}
void MovePanTilt::QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}
}  // namespace gazebo
