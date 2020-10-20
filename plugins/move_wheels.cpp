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

// This plugin is used to create the MoveWheels service and topic to move wheels motors in the Gazebo model of Robobo
// \author Esteban Esquivel

#include "include/robobo/move_wheels.h"
#include "ros/advertise_options.h"
#include "ros/advertise_service_options.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Int16.h"

namespace gazebo
{
// Register the plugin in Gazebo.
GZ_REGISTER_MODEL_PLUGIN(MoveWheels)

// Constructor
MoveWheels::MoveWheels()
{
}

// Deconstructor
MoveWheels::~MoveWheels()
{
}

// When plugin is loaded
void MoveWheels::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
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

  // Create MoveWheels service
  ros::AdvertiseServiceOptions advSerOpt = ros::AdvertiseServiceOptions::create<robobo_msgs::MoveWheels>(
      "moveWheels", boost::bind(&MoveWheels::ServiceCallback, this, _1, _2), ros::VoidPtr(), &this->rosQueue);
  moveWheelsService = rosNode->advertiseService(advSerOpt);

  // MoveWheels Subscriber
  ros::SubscribeOptions subOpt = ros::SubscribeOptions::create<robobo_msgs::MoveWheelsCommand>(
      "moveWheels", 10, boost::bind(&MoveWheels::TopicCallback, this, _1), ros::VoidPtr(), &this->rosQueue);
  moveWheelsSubscriber = rosNode->subscribe(subOpt);

  // BlockId Publisher
  blockPub = rosNode->advertise<std_msgs::Int16>("unlock/move", 10);

  // Queue thread
  this->rosQueueThread = std::thread(std::bind(&MoveWheels::QueueThread, this));
}

bool MoveWheels::ServiceCallback(robobo_msgs::MoveWheels::Request &req, robobo_msgs::MoveWheels::Response &res)
{
  int8_t lspeed = (req.lspeed.data < -100) ? -100 : (100 < req.lspeed.data) ? 100 : req.lspeed.data;
  int8_t rspeed = (req.rspeed.data < -100) ? -100 : (100 < req.rspeed.data) ? 100 : req.rspeed.data;
  float time = req.time.data / 1000.0;
  moveWheelsThread =
      std::thread(std::bind(&MoveWheels::HandleMoveWheels, this, lspeed, rspeed, time, req.unlockid.data));
  moveWheelsThread.detach();
  return true;
}

void MoveWheels::TopicCallback(const boost::shared_ptr<robobo_msgs::MoveWheelsCommand const> &req)
{
  int8_t lspeed = (req->lspeed.data < -100) ? -100 : (100 < req->lspeed.data) ? 100 : req->lspeed.data;
  int8_t rspeed = (req->rspeed.data < -100) ? -100 : (100 < req->rspeed.data) ? 100 : req->rspeed.data;
  float time = req->time.data / 1000.0;
  moveWheelsThread =
      std::thread(std::bind(&MoveWheels::HandleMoveWheels, this, lspeed, rspeed, time, req->unlockid.data));
  moveWheelsThread.detach();
}

// Function that controlls the wheels, this should be called from the callbacks
// TODO: calculate again the formula to simulate the velocity.
void MoveWheels::HandleMoveWheels(int8_t lwp, int8_t rwp, float time, int16_t blockid)
{
  double lspeedGazebo = 0, rspeedGazebo = 0;
  int8_t abs_lwp = abs(lwp), abs_rwp = abs(rwp);

  // Set torque/force
  model->GetJoint("right_motor")->SetParam("fmax", 0, 0.5);
  model->GetJoint("left_motor")->SetParam("fmax", 0, 0.5);

  // Calculate joints target velocity
  if (lwp != 0)
  {
    lspeedGazebo =
        ((-4.625E-05 * pow(abs_lwp, 3) + 5.219E-03 * pow(abs_lwp, 2) + 6.357 * abs_lwp + 5.137E+01) +
         (-3.253E-04 * pow(abs_lwp, 3) + 4.285E-02 * pow(abs_lwp, 2) + -2.064 * abs_lwp - 1.770E+01) / time) *
        M_PI / 180;
    if (lwp < 0)
      lspeedGazebo = -lspeedGazebo;
  }

  if (rwp != 0)
  {
    rspeedGazebo =
        ((-4.625E-05 * pow(abs_rwp, 3) + 5.219E-03 * pow(abs_rwp, 2) + 6.357 * abs_rwp + 5.137E+01) +
         (-3.253E-04 * pow(abs_rwp, 3) + 4.285E-02 * pow(abs_rwp, 2) + -2.064 * abs_rwp - 1.770E+01) / time) *
        M_PI / 180;
    if (rwp < 0)
      rspeedGazebo = -rspeedGazebo;
  }

  // Set joints velocity
  model->GetJoint("left_motor")->SetParam("vel", 0, lspeedGazebo);
  model->GetJoint("right_motor")->SetParam("vel", 0, rspeedGazebo);
  usleep(time * 1E6);
  model->GetJoint("left_motor")->SetParam("vel", 0, double(0));
  model->GetJoint("right_motor")->SetParam("vel", 0, double(0));
  model->GetJoint("right_motor")->SetParam("fmax", 0, 0);
  model->GetJoint("left_motor")->SetParam("fmax", 0, 0);
  if (blockPub.getNumSubscribers() > 0)
  {
    std_msgs::Int16 unblockId;
    unblockId.data = blockid;
    blockPub.publish(unblockId);
    ros::spinOnce();
  }
}

void MoveWheels::QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}
}  // namespace gazebo
