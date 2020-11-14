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

#include "include/robobo/battery_consumer.hh"
#include "gazebo/common/Battery.hh"
#include "gazebo/physics/physics.hh"
#include "include/robobo/ROS_debugging.h"


using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(BatteryConsumerPlugin);

BatteryConsumerPlugin::BatteryConsumerPlugin() : consumerId(-1)
{
}

BatteryConsumerPlugin::~BatteryConsumerPlugin()
{
    if (this->battery && this->consumerId !=-1)
        this->battery->RemoveConsumer(this->consumerId);
}

void BatteryConsumerPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // TODO: checking whether these elements exists

    // check if the ros is up!
    if (!ros::isInitialized()){
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, _sdf->Get<std::string>("ros_node"), ros::init_options::NoSigintHandler);
    }

    this->model = _model;
    this->world = _model->GetWorld();

    std::string linkName = _sdf->Get<std::string>("link_name");
    this->link = _model->GetLink(linkName);

    // Create battery
    std::string batteryName = _sdf->Get<std::string>("battery_name");
    this->battery = this->link->Battery(batteryName);

    // Add consumer and sets its power load
    this->powerLoad = _sdf->Get<double>("power_load");
    this->consumerId = this->battery->AddConsumer();
    this->battery->SetPowerLoad(this->consumerId, powerLoad);

    // Create ros node and publish stuff there!
    this->rosNode.reset(new ros::NodeHandle(model->GetName()));

    this->set_power_load = this->rosNode->advertiseService(this->model->GetName() + "/"+ linkName+"/set_power_load", &BatteryConsumerPlugin::SetConsumerPowerLoad, this);


    //ROS_GREEN_STREAM("Consumer loaded");

}

void BatteryConsumerPlugin::Init()
{
    //ROS_GREEN_STREAM("Consumer is initialized");
}

void BatteryConsumerPlugin::Reset()
{
    //ROS_GREEN_STREAM("Consumer is reset");
}

bool BatteryConsumerPlugin::SetConsumerPowerLoad(robobo_msgs::SetLoad::Request &req,
                                                 robobo_msgs::SetLoad::Response &res)
{
    lock.lock();
    double load = this->powerLoad;
    this->powerLoad = req.power_load;
    this->battery->SetPowerLoad(this->consumerId, this->powerLoad);

    //ROS_GREEN_STREAM("Power load of consumer has changed to: " << this->powerLoad);

    lock.unlock();
    res.result = true;
    return true;
}
