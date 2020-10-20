#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Battery.hh"
#include "gazebo/physics/physics.hh"
#include "include/robobo/battery_discharge.hh"
#include "std_msgs/Int32.h"
#include "include/robobo/ROS_debugging.h"


using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(BatteryPlugin);

BatteryPlugin::BatteryPlugin()
{
    this->c = 0.0;
    this->r = 0.0;
    this->tau = 0.0;

    this->e0 = 0.0;
    this->e1 = 0.0;

    this->q0 = 0.0;
    this->q = 0.0;
    this->qt = 0.0;

    this->iraw = 0.0;
    this->ismooth = 0.0;

    ROS_INFO_STREAM("BRASS CP1 battery is constructed.");
}

BatteryPlugin::~BatteryPlugin()
{
    this->rosNode->shutdown();
}

void BatteryPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    // check if the ros is up!
    if (!ros::isInitialized()){
        ROS_INFO_STREAM("Initializing ROS...");
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, _sdf->Get<std::string>("ros_node"), ros::init_options::NoSigintHandler);
    }

    model = _model;
    this->world = _model->GetWorld();

    this->sim_time_now = this->world->GetSimTime().Double();

    std::string robot_namespace = "";
    if (_sdf->HasElement("robotNamespace")) {
        robot_namespace = _sdf->GetElement("robotNamespace")->Get<std::string>();
    }
    // Create node handler and publish stuff there!
    this->rosNode.reset(new ros::NodeHandle(robot_namespace));

    if (this->rosNode->ok())
    {
        //ROS_GREEN_STREAM("ROS node is up");
    }

    std::string batteryName = _sdf->Get<std::string>("battery_name");

    // Publish a topic for charge level
    this->charge_state = this->rosNode->advertise<std_msgs::Int32>("battery/"+batteryName, 1);

    std::string linkName = _sdf->Get<std::string>("link_name");
    this->link = this->model->GetLink(linkName);

    this->e0 = _sdf->Get<double>("constant_coef");
    this->e1 = _sdf->Get<double>("linear_coef");
    this->qt = _sdf->Get<double>("charge_rate");
    this->c = _sdf->Get<double>("capacity");
    this->r = _sdf->Get<double>("resistance");
    this->tau = _sdf->Get<double>("smooth_current_tau");
    this->q0 = _sdf->Get<double>("initial_charge");

    

    if (this->link->BatteryCount() > 0) {
        // Creates the battery
        this->battery = this->link->Battery(batteryName);
        //ROS_GREEN_STREAM("Created a battery");
    }
    else
    {
        //ROS_RED_STREAM("There is no battery specification in the link");
    };


    // Specifying a custom update function
    this->battery->SetUpdateFunc(std::bind(&BatteryPlugin::OnUpdateVoltage, this, std::placeholders::_1));

    this->sim_time_now = this->world->GetSimTime().Double();

    //ROS_GREEN_STREAM("Plugin is fully loaded.");
}


void BatteryPlugin::Init()
{
    //ROS_GREEN_STREAM("Init Battery " << this->q0);
    this->q = this->q0;
}

void BatteryPlugin::Reset()
{
    this->iraw = 0.0;
    this->ismooth = 0.0;
    this->Init();
}

double BatteryPlugin::OnUpdateVoltage(const common::BatteryPtr &_battery)
{
    double dt = this->world->GetPhysicsEngine()->GetMaxStepSize();
    double totalpower = 0.0;
    double k = dt / this->tau;

    if (fabs(_battery->Voltage())<1e-3)
    {

        std_msgs::Int32 charge_msg;
        charge_msg.data = 5;
        lock.lock();
        this->charge_state.publish(charge_msg);
        lock.unlock();
        return 0.0;
    }

    for (auto powerLoad : _battery->PowerLoads())
        totalpower += powerLoad.second;

    this->iraw = totalpower / _battery->Voltage();

    this->ismooth = this->ismooth + k * (this->iraw - this->ismooth);
    
    this->q = this->q - GZ_SEC_TO_HOUR(dt * this->ismooth);

    this->sim_time_now = this->world->GetSimTime().Double();

    this->et = this->e0 + this->e1 * (1 - this->q / this->c) - this->r * this->ismooth;

    //Turn off the motor
    if (this->q <= 0)
    {
        this->sim_time_now = this->world->GetSimTime().Double();


        this->q = 0;
    }
    else if (this->q >= this->c)
    {
        this->q = this->c;
    }

    std_msgs::Int32 charge_msg;
    charge_msg.data = this->q/this->c*100;

    lock.lock();
    this->charge_state.publish(charge_msg);
    lock.unlock();

    return et;
}



