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

/* Derived from the work :
/* Copyright [2015] [Alessandro Settimi]
 *
 * email: ale.settimi@gmail.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.*/

#include "include/robobo/robobo_imu_sensor.h"

namespace gazebo
{
GZ_REGISTER_SENSOR_PLUGIN(RoboboImuSensor)
RoboboImuSensor::RoboboImuSensor() : SensorPlugin()
{
  accelerometer_data = ignition::math::Vector3d(0, 0, 0);
  // gyroscope_data = ignition::math::Vector3d(0, 0, 0);
  orientation = ignition::math::Quaterniond(1, 0, 0, 0);
  seed = 0;
  sensor = NULL;
}

RoboboImuSensor::~RoboboImuSensor()
{
  if (connection.get())
  {
    connection.reset();
    connection = event::ConnectionPtr();
  }

  node->shutdown();
}

void RoboboImuSensor::Load(sensors::SensorPtr sensor_, sdf::ElementPtr sdf_)
{
  sdf = sdf_;
  sensor = dynamic_cast<sensors::ImuSensor *>(sensor_.get());

  if (sensor == NULL)
  {
    ROS_FATAL("Error: Sensor pointer is NULL!");
    return;
  }

  sensor->SetActive(true);

  robot_namespace = GetRobotNamespace(sensor_, sdf, "IMU Sensor");

  if (!LoadParameters())
  {
    ROS_FATAL("Error Loading Parameters!");
    return;
  }

  if (!ros::isInitialized())
  {
    ROS_FATAL("ROS has not been initialized!");
    return;
  }

  node = new ros::NodeHandle(this->robot_namespace);

  accelPub = node->advertise<geometry_msgs::Accel>(robot_namespace + "accel", 1);
  orientationPub = node->advertise<geometry_msgs::Quaternion>(robot_namespace + "orientation", 1);

  connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&RoboboImuSensor::UpdateChild, this, _1));

  last_time = sensor->LastUpdateTime();
}

void RoboboImuSensor::UpdateChild(const common::UpdateInfo & /*_info*/)
{
  common::Time current_time = sensor->LastUpdateTime();
  ignition::math::Vector3d newAccelData = sensor->LinearAcceleration();
  ignition::math::Quaterniond newOrientationData = offset.Rot() * sensor->Orientation();

  if (update_rate > 0 && (current_time - last_time).Double() < 1.0 / update_rate)  // update rate check
    return;

  if (accelPub.getNumSubscribers() > 0 && accelerometer_data.X() != newAccelData.X() &&
      accelerometer_data.Y() != newAccelData.Y() && accelerometer_data.Z() != newAccelData.Z())
  {
    accelerometer_data = newAccelData;
    accel_msg.linear.x = accelerometer_data.X() + GuassianKernel(0, gaussian_noise);
    accel_msg.linear.y = accelerometer_data.Y() + GuassianKernel(0, gaussian_noise);
    accel_msg.linear.z = accelerometer_data.Z() + GuassianKernel(0, gaussian_noise);

    // publishing data
    accelPub.publish(accel_msg);
  }
  if (orientationPub.getNumSubscribers() > 0 && orientation.X() != newOrientationData.X() &&
      orientation.Y() != newOrientationData.Y() && orientation.Z() != newOrientationData.Z() &&
      orientation.W() != newOrientationData.W())
  {
    // gyroscope_data = sensor->AngularVelocity();

    orientation = newOrientationData;  // applying offsets to the orientation measurement
    // Guassian noise is applied to all measurements
    orient_msg.x = orientation.X() + GuassianKernel(0, gaussian_noise);
    orient_msg.y = orientation.Y() + GuassianKernel(0, gaussian_noise);
    orient_msg.z = orientation.Z() + GuassianKernel(0, gaussian_noise);
    orient_msg.w = orientation.W() + GuassianKernel(0, gaussian_noise);
    orientationPub.publish(orient_msg);
  }

  ros::spinOnce();
  last_time = current_time;
}

double RoboboImuSensor::GuassianKernel(double mu, double sigma)
{
  // generation of two normalized uniform random variables
  double U1 = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);
  double U2 = static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX);

  // using Box-Muller transform to obtain a varaible with a standard normal distribution
  double Z0 = sqrt(-2.0 * ::log(U1)) * cos(2.0 * M_PI * U2);

  // scaling
  Z0 = sigma * Z0 + mu;
  return Z0;
}

bool RoboboImuSensor::LoadParameters()
{
  // loading parameters from the sdf file

  // NAMESPACE
  // std::string scoped_name = sensor->ParentName();
    // std::size_t it = scoped_name.find(":");
    // robot_namespace = "/" +scoped_name.substr(0,it)+"/";

  // UPDATE RATE
  if (sdf->HasElement("updateRateHZ"))
  {
    update_rate = sdf->Get<double>("updateRateHZ");
    ROS_INFO_STREAM("<updateRateHZ> set to: " << update_rate);
  }
  else
  {
    update_rate = 1.0;
    ROS_WARN_STREAM("missing <updateRateHZ>, set to default: " << update_rate);
  }

  // NOISE
  if (sdf->HasElement("gaussianNoise"))
  {
    gaussian_noise = sdf->Get<double>("gaussianNoise");
    ROS_INFO_STREAM("<gaussianNoise> set to: " << gaussian_noise);
  }
  else
  {
    gaussian_noise = 0.0;
    ROS_WARN_STREAM("missing <gaussianNoise>, set to default: " << gaussian_noise);
  }

  // POSITION OFFSET, UNUSED
  if (sdf->HasElement("xyzOffset"))
  {
    offset.Pos() = sdf->Get<ignition::math::Vector3d>("xyzOffset");
    ROS_INFO_STREAM("<xyzOffset> set to: " << offset.Pos()[0] << ' ' << offset.Pos()[1] << ' ' << offset.Pos()[2]);
  }
  else
  {
    offset.Pos() = ignition::math::Vector3d(0, 0, 0);
    ROS_WARN_STREAM("missing <xyzOffset>, set to default: " << offset.Pos()[0] << ' ' << offset.Pos()[1] << ' '
                                                            << offset.Pos()[2]);
  }

  // ORIENTATION OFFSET
  if (sdf->HasElement("rpyOffset"))
  {
    offset.Rot() = ignition::math::Quaterniond(sdf->Get<ignition::math::Vector3d>("rpyOffset"));
    ROS_INFO_STREAM("<rpyOffset> set to: " << offset.Rot().Roll() << ' ' << offset.Rot().Pitch() << ' '
                                           << offset.Rot().Yaw());
  }
  else
  {
    offset.Rot() = ignition::math::Quaterniond::Identity;
    ROS_WARN_STREAM("missing <rpyOffset>, set to default: " << offset.Rot().Roll() << ' ' << offset.Rot().Pitch() << ' '
                                                            << offset.Rot().Yaw());
  }

  return true;
}

}  // namespace gazebo
