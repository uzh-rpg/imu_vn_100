/*
 * Copyright [2015] [Ke Sun]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <imu_vn_100/imu_vn_100.h>

namespace imu_vn_100 {

// LESS HACK IS STILL HACK
ImuVn100* imu_vn_100_ptr;



using sensor_msgs::Imu;
using sensor_msgs::MagneticField;
using sensor_msgs::FluidPressure;
using sensor_msgs::Temperature;

void RosVector3FromVnVector3(geometry_msgs::Vector3& ros_vec3,
                             const VnVector3& vn_vec3);
void RosQuaternionFromVnQuaternion(geometry_msgs::Quaternion& ros_quat,
                                   const VnQuaternion& vn_quat);
void FillImuMessage(sensor_msgs::Imu& imu_msg,
                    const VnDeviceCompositeData& data, bool binary_output);

geometry_msgs::Vector3 rotateVector(const VnMatrix3x3 rotation_Matrix, const geometry_msgs::Vector3 input_vector)
{
  geometry_msgs::Vector3 rotated_vector;
  rotated_vector.x = rotation_Matrix.c00*input_vector.x + rotation_Matrix.c01*input_vector.y + rotation_Matrix.c02*input_vector.z;
  rotated_vector.y = rotation_Matrix.c10*input_vector.x + rotation_Matrix.c11*input_vector.y + rotation_Matrix.c12*input_vector.z;
  rotated_vector.z = rotation_Matrix.c20*input_vector.x + rotation_Matrix.c21*input_vector.y + rotation_Matrix.c22*input_vector.z;
  return rotated_vector;
}

void AsyncListener(void* sender, VnDeviceCompositeData* data) {
  imu_vn_100_ptr->applyImuFilter(*data);
  imu_vn_100_ptr->PublishData(*data);
}

void ImuFilter::initFilter(float sample_freq, float cutoff_freq)
{

  ROS_INFO("init filters with a sampling frequency of %f and a cutoff of %f\n",sample_freq,cutoff_freq);
  filter_acceleration_x_.set_cutoff_frequency(sample_freq,cutoff_freq);
  filter_acceleration_y_.set_cutoff_frequency(sample_freq,cutoff_freq);
  filter_acceleration_z_.set_cutoff_frequency(sample_freq,cutoff_freq);
  filter_gyro_x_.set_cutoff_frequency(sample_freq,cutoff_freq);
  filter_gyro_y_.set_cutoff_frequency(sample_freq,cutoff_freq);
  filter_gyro_z_.set_cutoff_frequency(sample_freq,cutoff_freq);

  filter_acceleration_x_.reset(0.0);
  filter_acceleration_y_.reset(0.0);
  filter_acceleration_z_.reset(0.0);
  filter_gyro_x_.reset(0.0);
  filter_gyro_y_.reset(0.0);
  filter_gyro_z_.reset(0.0);
}

void ImuFilter::updateFilterAcceleration(float acceleration_x, float acceleration_y, float acceleration_z)
{
  current_acceleration_x_ = filter_acceleration_x_.apply(acceleration_x);
  current_acceleration_y_ = filter_acceleration_y_.apply(acceleration_y);
  current_acceleration_z_ = filter_acceleration_z_.apply(acceleration_z);
}

void ImuFilter::updatefilterGyro(float gyro_x, float gyro_y, float gyri_z)
{
  current_gyro_x_ = filter_gyro_x_.apply(gyro_x);
  current_gyro_y_ = filter_gyro_y_.apply(gyro_y);
  current_gyro_z_ = filter_gyro_z_.apply(gyri_z);
}

geometry_msgs::Vector3 ImuFilter::getCurrentAccleration()
{
  geometry_msgs::Vector3 acceleration;
  acceleration.x = current_acceleration_x_;
  acceleration.y = current_acceleration_y_;
  acceleration.z = current_acceleration_z_;
  return acceleration;
}

geometry_msgs::Vector3 ImuFilter::getCurrentGyro()
{
  geometry_msgs::Vector3 gyro;
  gyro.x = current_gyro_x_;
  gyro.y = current_gyro_y_;
  gyro.z = current_gyro_z_;
  return gyro;
}

constexpr int ImuVn100::kBaseImuRate;
constexpr int ImuVn100::kDefaultImuRate;
constexpr int ImuVn100::kDefaultSyncOutRate;
constexpr float ImuVn100::kDefaultCutoff;

void ImuVn100::SyncInfo::Update(const unsigned sync_count,
                                const ros::Time& sync_time) {
  if (rate <= 0) return;

  if (count != sync_count) {
    count = sync_count;
    time = sync_time;
  }
}

bool ImuVn100::SyncInfo::SyncEnabled() const { return rate > 0; }

void ImuVn100::SyncInfo::FixSyncRate() {
  // Check the sync out rate
  if (SyncEnabled()) {
    if (ImuVn100::kBaseImuRate % rate != 0) {
      rate = ImuVn100::kBaseImuRate / (ImuVn100::kBaseImuRate / rate);
      ROS_INFO("Set SYNC_OUT_RATE to %d", rate);
    }
    skip_count =
        (std::floor(ImuVn100::kBaseImuRate / static_cast<double>(rate) +
                    0.5f)) -
                    1;

    if (pulse_width_us > 10000) {
      ROS_INFO("Sync out pulse with is over 10ms. Reset to 1ms");
      pulse_width_us = 1000;
    }
    rate_double = rate;
  }

  ROS_INFO("Sync out rate: %d", rate);
}

ImuVn100::ImuVn100(const ros::NodeHandle& pnh)
: pnh_(pnh),
  port_(std::string("/dev/ttyUSB0")),
  baudrate_(921600),
  frame_id_(std::string("imu")) {
  Initialize();
  imu_vn_100_ptr = this;
}

ImuVn100::~ImuVn100() { Disconnect(); }

void ImuVn100::FixImuRate() {
  if (imu_rate_ <= 0) {
    ROS_WARN("Imu rate %d is < 0. Set to %d", imu_rate_, kDefaultImuRate);
    imu_rate_ = kDefaultImuRate;
  }

  if (kBaseImuRate % imu_rate_ != 0) {
    int imu_rate_old = imu_rate_;
    imu_rate_ = kBaseImuRate / (kBaseImuRate / imu_rate_old);
    ROS_WARN("Imu rate %d cannot evenly decimate base rate %d, reset to %d",
             imu_rate_old, kBaseImuRate, imu_rate_);
  }
}

void ImuVn100::LoadParameters() {
  pnh_.param<std::string>("port", port_, std::string("/dev/ttyUSB0"));
  pnh_.param<std::string>("frame_id", frame_id_, pnh_.getNamespace());
  pnh_.param("baudrate", baudrate_, 115200);
  pnh_.param("imu_rate", imu_rate_, kDefaultImuRate);

  pnh_.param("enable_mag", enable_mag_, true);
  pnh_.param("enable_pres", enable_pres_, true);
  pnh_.param("enable_temp", enable_temp_, true);

  pnh_.param("sync_rate", sync_info_.rate, kDefaultSyncOutRate);
  pnh_.param("sync_pulse_width_us", sync_info_.pulse_width_us, 1000);

  pnh_.param("binary_output", binary_output_, true);
  pnh_.param("imu_cutoff_frequency", imu_cutoff_freq_, kDefaultCutoff);
  ROS_INFO("Cutoff is set to %f", imu_cutoff_freq_);


  pnh_.param("accelerometer_bias_x", accelerometer_bias_x_, (float) 0.0);
  pnh_.param("accelerometer_bias_y", accelerometer_bias_y_, (float) 0.0);
  pnh_.param("accelerometer_bias_z", accelerometer_bias_z_, (float) 0.0);

  pnh_.param("gyro_bias_x", gyro_bias_x_, (float) 0.0);
  pnh_.param("gyro_bias_y", gyro_bias_y_, (float) 0.0);
  pnh_.param("gyro_bias_z", gyro_bias_z_, (float) 0.0);

  pnh_.param("c00", rotation_body_imu_.c00, 1.0);
  pnh_.param("c01", rotation_body_imu_.c01, 0.0);
  pnh_.param("c02", rotation_body_imu_.c02, 0.0);
  pnh_.param("c10", rotation_body_imu_.c10, 0.0);
  pnh_.param("c11", rotation_body_imu_.c11, -1.0);
  pnh_.param("c12", rotation_body_imu_.c12, 0.0);
  pnh_.param("c20", rotation_body_imu_.c20, 0.0);
  pnh_.param("c21", rotation_body_imu_.c21, 0.0);
  pnh_.param("c22", rotation_body_imu_.c22, -1.0);

  FixImuRate();
  sync_info_.FixSyncRate();
}

void ImuVn100::CreateDiagnosedPublishers() {
  imu_rate_double_ = imu_rate_;
  pd_imu_.Create<Imu>(pnh_, "imu", updater_, imu_rate_double_);
  if (enable_mag_) {
    pd_mag_.Create<MagneticField>(pnh_, "magnetic_field", updater_,
                                  imu_rate_double_);
  }
  if (enable_pres_) {
    pd_pres_.Create<FluidPressure>(pnh_, "fluid_pressure", updater_,
                                   imu_rate_double_);
  }
  if (enable_temp_) {
    pd_temp_.Create<Temperature>(pnh_, "temperature", updater_,
                                 imu_rate_double_);
  }
}

void ImuVn100::Initialize() {
  LoadParameters();

  // initialize filters
  imu_filter_.initFilter(imu_rate_,imu_cutoff_freq_);

  ROS_DEBUG("Connecting to device");
  VnEnsure(vn100_connect(&imu_, port_.c_str(), 115200));
  ros::Duration(0.5).sleep();
  ROS_INFO("Connected to device at %s", port_.c_str());

  unsigned int old_baudrate;
  VnEnsure(vn100_getSerialBaudRate(&imu_, &old_baudrate));
  ROS_INFO("Default serial baudrate: %u", old_baudrate);

  ROS_INFO("Set serial baudrate to %d", baudrate_);
  VnEnsure(vn100_setSerialBaudRate(&imu_, baudrate_, true));

  ROS_DEBUG("Disconnecting the device");
  vn100_disconnect(&imu_);
  ros::Duration(0.5).sleep();

  ROS_DEBUG("Reconnecting to device");
  VnEnsure(vn100_connect(&imu_, port_.c_str(), baudrate_));
  ros::Duration(0.5).sleep();
  ROS_INFO("Connected to device at %s", port_.c_str());

  VnEnsure(vn100_getSerialBaudRate(&imu_, &old_baudrate));
  ROS_INFO("New serial baudrate: %u", old_baudrate);

  // Idle the device for intialization
  VnEnsure(vn100_pauseAsyncOutputs(&imu_, true));

  ROS_INFO("Fetching device info.");
  char model_number_buffer[30] = {0};
  int hardware_revision = 0;
  char serial_number_buffer[30] = {0};
  char firmware_version_buffer[30] = {0};

  VnEnsure(vn100_getModelNumber(&imu_, model_number_buffer, 30));
  ROS_INFO("Model number: %s", model_number_buffer);
  VnEnsure(vn100_getHardwareRevision(&imu_, &hardware_revision));
  ROS_INFO("Hardware revision: %d", hardware_revision);
  VnEnsure(vn100_getSerialNumber(&imu_, serial_number_buffer, 30));
  ROS_INFO("Serial number: %s", serial_number_buffer);
  VnEnsure(vn100_getFirmwareVersion(&imu_, firmware_version_buffer, 30));
  ROS_INFO("Firmware version: %s", firmware_version_buffer);

  if (sync_info_.SyncEnabled()) {
    ROS_INFO("Set Synchronization Control Register (id:32).");
    VnEnsure(vn100_setSynchronizationControl(
        &imu_, SYNCINMODE_COUNT, SYNCINEDGE_RISING, 0, SYNCOUTMODE_IMU_START,
        SYNCOUTPOLARITY_POSITIVE, sync_info_.skip_count,
        sync_info_.pulse_width_us * 1000, true));

    if (!binary_output_) {
      ROS_INFO("Set Communication Protocal Control Register (id:30).");
      VnEnsure(vn100_setCommunicationProtocolControl(
          &imu_, SERIALCOUNT_SYNCOUT_COUNT, SERIALSTATUS_OFF, SPICOUNT_NONE,
          SPISTATUS_OFF, SERIALCHECKSUM_8BIT, SPICHECKSUM_8BIT, ERRORMODE_SEND,
          true));
    }
  }

  CreateDiagnosedPublishers();

  auto hardware_id = std::string("vn100-") + std::string(model_number_buffer) +
      std::string(serial_number_buffer);
  updater_.setHardwareID(hardware_id);

}

void ImuVn100::Stream(bool async) {
  // Pause the device first
  VnEnsure(vn100_pauseAsyncOutputs(&imu_, true));

  if (async) {
    VnEnsure(vn100_setAsynchronousDataOutputType(&imu_, VNASYNC_OFF, true));

    if (binary_output_) {
      // Set the binary output data type and data rate
      VnEnsure(vn100_setBinaryOutput1Configuration(
          &imu_, BINARY_ASYNC_MODE_SERIAL_2, kBaseImuRate / imu_rate_,
          BG1_QTN | BG1_IMU | BG1_MAG_PRES | BG1_SYNC_IN_CNT,
          // BG1_IMU,
          BG3_NONE, BG5_NONE, true));
    } else {
      // Set the ASCII output data type and data rate
      // ROS_INFO("Configure the output data type and frequency (id: 6 & 7)");
      VnEnsure(vn100_setAsynchronousDataOutputType(&imu_, VNASYNC_VNIMU, true));
    }

    // Add a callback function for new data event
    VnEnsure(vn100_registerAsyncDataReceivedListener(&imu_, &AsyncListener));

    ROS_INFO("Setting IMU rate to %d", imu_rate_);
    VnEnsure(vn100_setAsynchronousDataOutputFrequency(&imu_, imu_rate_, true));
  } else {
    // Mute the stream
    ROS_DEBUG("Mute the device");
    VnEnsure(vn100_setAsynchronousDataOutputType(&imu_, VNASYNC_OFF, true));
    // Remove the callback function for new data event
    VnEnsure(vn100_unregisterAsyncDataReceivedListener(&imu_, &AsyncListener));
  }

  // Resume the device
  VnEnsure(vn100_resumeAsyncOutputs(&imu_, true));
}

void ImuVn100::Resume(bool need_reply) {
  vn100_resumeAsyncOutputs(&imu_, need_reply);
}

void ImuVn100::Idle(bool need_reply) {
  vn100_pauseAsyncOutputs(&imu_, need_reply);
}

void ImuVn100::Disconnect() {
  // TODO: why reset the device?
  vn100_reset(&imu_);
  vn100_disconnect(&imu_);
}

void ImuVn100::applyImuFilter(const VnDeviceCompositeData& data) {
  // Attention: Acceleration and angular rates are swapped!
  imu_filter_.updateFilterAcceleration(data.angularRateUncompensated.c0,
                                       data.angularRateUncompensated.c1,
                                       data.angularRateUncompensated.c2);
  imu_filter_.updatefilterGyro(data.accelerationUncompensated.c0,
                               data.accelerationUncompensated.c1,
                               data.accelerationUncompensated.c2);

}

void ImuVn100::PublishData(const VnDeviceCompositeData& data) {
  sensor_msgs::Imu imu_msg;
  imu_msg.header.stamp = ros::Time::now();
  imu_msg.header.frame_id = frame_id_;

  FillImuMessage(imu_msg, data, binary_output_);
  pd_imu_.Publish(imu_msg);

  if (enable_mag_) {
    sensor_msgs::MagneticField mag_msg;
    mag_msg.header = imu_msg.header;
    RosVector3FromVnVector3(mag_msg.magnetic_field, data.magnetic);
    pd_mag_.Publish(mag_msg);
  }

  if (enable_pres_) {
    sensor_msgs::FluidPressure pres_msg;
    pres_msg.header = imu_msg.header;
    pres_msg.fluid_pressure = data.pressure;
    pd_pres_.Publish(pres_msg);
  }

  if (enable_temp_) {
    sensor_msgs::Temperature temp_msg;
    temp_msg.header = imu_msg.header;
    temp_msg.temperature = data.temperature;
    pd_temp_.Publish(temp_msg);
  }

  sync_info_.Update(data.syncInCnt, imu_msg.header.stamp);

  updater_.update();
}

geometry_msgs::Vector3 ImuVn100::getFilteredAccelerationInBodyFrame()
{
  geometry_msgs::Vector3 biased_acceleration = rotateVector(rotation_body_imu_,imu_filter_.getCurrentAccleration());
  geometry_msgs::Vector3 unbiased_acceleration;
  unbiased_acceleration.x = biased_acceleration.x - accelerometer_bias_x_;
  unbiased_acceleration.y = biased_acceleration.y - accelerometer_bias_y_;
  unbiased_acceleration.z = biased_acceleration.z - accelerometer_bias_z_;

  return unbiased_acceleration;
}

geometry_msgs::Vector3 ImuVn100::getFilteredGyroInBodyFrame()
{
  geometry_msgs::Vector3 biased_gyro = rotateVector(rotation_body_imu_,imu_filter_.getCurrentGyro());
  geometry_msgs::Vector3 unbiased_gyro;
  unbiased_gyro.x = biased_gyro.x - gyro_bias_x_;
  unbiased_gyro.y = biased_gyro.y - gyro_bias_y_;
  unbiased_gyro.z = biased_gyro.z - gyro_bias_z_;

  return unbiased_gyro;
}

void VnEnsure(const VnErrorCode& error_code) {
  if (error_code == VNERR_NO_ERROR) return;

  switch (error_code) {
    case VNERR_UNKNOWN_ERROR:
      throw std::runtime_error("VN: Unknown error");
    case VNERR_NOT_IMPLEMENTED:
      throw std::runtime_error("VN: Not implemented");
    case VNERR_TIMEOUT:
      ROS_WARN("Opertation time out");
      break;
    case VNERR_SENSOR_INVALID_PARAMETER:
      ROS_WARN("VN: Sensor invalid paramter");
      break;
    case VNERR_INVALID_VALUE:
      ROS_WARN("VN: Invalid value");
      break;
    case VNERR_FILE_NOT_FOUND:
      ROS_WARN("VN: File not found");
      break;
    case VNERR_NOT_CONNECTED:
      throw std::runtime_error("VN: not connected");
    case VNERR_PERMISSION_DENIED:
      throw std::runtime_error("VN: Permission denied");
    default:
      ROS_WARN("Unhandled error type");
  }
}

void RosVector3FromVnVector3(geometry_msgs::Vector3& ros_vec3,
                             const VnVector3& vn_vec3) {
  ros_vec3.x = vn_vec3.c0;
  ros_vec3.y = vn_vec3.c1;
  ros_vec3.z = vn_vec3.c2;
}

void RosQuaternionFromVnQuaternion(geometry_msgs::Quaternion& ros_quat,
                                   const VnQuaternion& vn_quat) {
  ros_quat.x = vn_quat.x;
  ros_quat.y = vn_quat.y;
  ros_quat.z = vn_quat.z;
  ros_quat.w = vn_quat.w;
}

void FillImuMessage(sensor_msgs::Imu& imu_msg,
                    const VnDeviceCompositeData& data, bool binary_output) {
  if (binary_output) {
    RosQuaternionFromVnQuaternion(imu_msg.orientation, data.quaternion);
    // NOTE: The IMU angular velocity and linear acceleration outputs are
    // swapped. And also why are they different?
    imu_msg.linear_acceleration = imu_vn_100_ptr->getFilteredAccelerationInBodyFrame();
    imu_msg.angular_velocity = imu_vn_100_ptr->getFilteredGyroInBodyFrame();

    //RosVector3FromVnVector3(imu_msg.angular_velocity,
    //                        data.accelerationUncompensated);
    //RosVector3FromVnVector3(imu_msg.linear_acceleration,
    //                        data.angularRateUncompensated);

  } else {
    RosVector3FromVnVector3(imu_msg.linear_acceleration, data.acceleration);
    RosVector3FromVnVector3(imu_msg.angular_velocity, data.angularRate);
  }
}

}  //  namespace imu_vn_100
