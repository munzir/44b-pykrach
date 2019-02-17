/*
 * Copyright (c) 2018, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @file krach.cpp
 * @author Munzir Zafar
 * @date-created Dec 4, 2018
 * @date-modified Feb 17, 2019
 * @brief Ach interface for controlling krang
 */

#include "pykrach/krach.h"

#include <somatic.h>  // has correct order of other includes, SOMATIC_WAIST_LAST_UNPACK, SOMATIC_PACK_SEND

#include <ach.h>           // ach_channel_t, ach_status_t, ach_result_to_string
#include <amino.h>         // aa_tm: _add, _sec2timespec(); aa_mem_ion_release()
#include <somatic.pb-c.h>  // SOMATIC__: EVENT, MOTOR, WAIST_MODE; somatic__anymsgtype__action(); Somatic__Waist: Mode, Cmd; SOMATIC__SIM_CMD__CODE__: STEP, RESET
#include <somatic/daemon.h>  // somatic_d: _init(), _event(), _destroy(), _channel...()
#include <somatic/motor.h>  // somati_motor: _t, _init(), _destroy(), _cmd(), _halt(), _reset(), _update()
#include <somatic/msg.h>  // somatic_anything_: alloc(), set(), free()

#include <assert.h>  // assert()
#include <string.h>  // strdup()
#include <time.h>    // struct timespec, clock_gettime(), CLOCK_MONOTONIC
#include <unistd.h>  // usleep()
#include <cstdio>    // fprintf()
#include <cstring>   // std::memset, strcpy
#include <string>    // std::string
#include <vector>    // std::vector

#include <boost/python.hpp>  // boost::python::dict
#include <boost/python/stl_iterator.hpp>

template <class T>
boost::python::list std_vector_to_py_list(const std::vector<T>& v) {
  boost::python::object get_iter = boost::python::iterator<std::vector<T> >();
  boost::python::object iter = get_iter(v);
  boost::python::list l(iter);
  return l;
}

template <typename T>
inline std::vector<T> to_std_vector(const boost::python::object& iterable) {
  return std::vector<T>(boost::python::stl_input_iterator<T>(iterable),
                        boost::python::stl_input_iterator<T>());
}

InterfaceContext::InterfaceContext(const std::string daemon_identifier) {
  // Initialize the daemon_
  std::memset(&daemon_opts_, 0, sizeof(daemon_opts_));
  daemon_opts_.ident = strdup(daemon_identifier.c_str());
  daemon_opts_.skip_sighandler = true;
  std::memset(&daemon_, 0, sizeof(daemon_));
  somatic_d_init(&daemon_, &daemon_opts_);

  // Send a "running" notice on the event channel
  somatic_d_event(&daemon_, SOMATIC__EVENT__PRIORITIES__NOTICE,
                  SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);
}

void InterfaceContext::Run() {
  // Free up the memory dynamically allocated when receiving commands
  aa_mem_region_release(&daemon_.memreg);
}

void InterfaceContext::Destroy() {
  std::cout << "interface context destroy" << std::endl;
  // Send a "stopping" notice on the event channel
  somatic_d_event(&daemon_, SOMATIC__EVENT__PRIORITIES__NOTICE,
                  SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

  // Destroy the daemon
  somatic_d_destroy(&daemon_);
}

MotorInterface::MotorInterface(InterfaceContext& interface_context,
                               std::string name,
                               std::string command_channel_name,
                               std::string state_channel_name, int num) {
  strcpy(name_, name.c_str());
  daemon_ = &interface_context.daemon_;
  n_ = num;

  motors_ = new somatic_motor_t();
  somatic_motor_init(daemon_, motors_, n_, command_channel_name.c_str(),
                     state_channel_name.c_str());

  // Set the min/max values for the pos/vel fields' valid and limit values
  for (int i = 0; i < n_; i++) {
    motors_->pos_valid_min[i] = -1024.1;
    motors_->pos_valid_max[i] = 1024.1;
    motors_->pos_limit_min[i] = -1024.1;
    motors_->pos_limit_max[i] = 1024.1;

    motors_->vel_valid_min[i] = -1024.1;
    motors_->vel_valid_max[i] = 1024.1;
    motors_->vel_limit_min[i] = -1024.1;
    motors_->vel_limit_max[i] = 1024.1;
  }

  // State update
  UpdateState();
  usleep(1e5);
}

void MotorInterface::Destroy() {
  std::cout << name_ << " destroy" << std::endl;
  ach_cancel(&motors_->cmd_chan, NULL);
  ach_close(&motors_->cmd_chan);
  ach_cancel(&motors_->state_chan, NULL);
  ach_close(&motors_->state_chan);
  somatic_motor_destroy(daemon_, motors_);
  delete motors_;
}

void MotorInterface::PositionCommand(const std::vector<double>& val) {
  somatic_motor_cmd(daemon_, motors_, SOMATIC__MOTOR_PARAM__MOTOR_POSITION,
                    &val[0], motors_->n, NULL);
}

void MotorInterface::VelocityCommand(const std::vector<double>& val) {
  somatic_motor_cmd(daemon_, motors_, SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY,
                    &val[0], motors_->n, NULL);
}

void MotorInterface::CurrentCommand(const std::vector<double>& val) {
  somatic_motor_cmd(daemon_, motors_, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT,
                    &val[0], motors_->n, NULL);
}

void MotorInterface::PositionCommandExt(const boost::python::list& val) {
  PositionCommand(to_std_vector<double>(val));
}

void MotorInterface::VelocityCommandExt(const boost::python::list& val) {
  VelocityCommand(to_std_vector<double>(val));
}

void MotorInterface::CurrentCommandExt(const boost::python::list& val) {
  CurrentCommand(to_std_vector<double>(val));
}

void MotorInterface::LockCommand() { somatic_motor_halt(daemon_, motors_); }

void MotorInterface::UnlockCommand() { somatic_motor_reset(daemon_, motors_); }

void MotorInterface::UpdateState() { somatic_motor_update(daemon_, motors_); }

std::vector<double> MotorInterface::GetPosition() {
  return std::vector<double>(motors_->pos, motors_->pos + n_);
}

std::vector<double> MotorInterface::GetVelocity() {
  return std::vector<double>(motors_->vel, motors_->vel + n_);
}

std::vector<double> MotorInterface::GetCurrent() {
  return std::vector<double>(motors_->cur, motors_->cur + n_);
}

boost::python::list MotorInterface::GetPositionExt() {
  return std_vector_to_py_list(GetPosition());
}

boost::python::list MotorInterface::GetVelocityExt() {
  return std_vector_to_py_list(GetVelocity());
}

boost::python::list MotorInterface::GetCurrentExt() {
  return std_vector_to_py_list(GetCurrent());
}

WaistInterface::WaistInterface(InterfaceContext& interface_context,
                               std::string name,
                               std::string command_channel_name,
                               std::string state_channel_name) {
  strcpy(name_, name.c_str());
  daemon_ = &interface_context.daemon_;
  waistd_command_channel_ = new ach_channel_t();
  somatic_d_channel_open(daemon_, waistd_command_channel_,
                         command_channel_name.c_str(), NULL);
  motors_ = new somatic_motor_t();
  somatic_motor_init(daemon_, motors_, 2, NULL, state_channel_name.c_str());
  usleep(1e5);

  waistd_command_msg_ = somatic_waist_cmd_alloc();

  for (int i = 0; i < 2; i++) {
    motors_->pos_valid_min[i] = -1024.1;
    motors_->pos_limit_min[i] = -1024.1;
    motors_->pos_valid_max[i] = 1024.1;
    motors_->pos_limit_max[i] = 1024.1;

    motors_->vel_valid_min[i] = -1024.1;
    motors_->vel_limit_min[i] = -1024.1;
    motors_->vel_valid_max[i] = 1024.1;
    motors_->vel_limit_max[i] = 1024.1;
  }

  somatic_motor_update(daemon_, motors_);
  usleep(1e5);
}

void WaistInterface::Destroy() {
  std::cout << name_ << " destroy" << std::endl;
  Stop();
  somatic_waist_cmd_free(waistd_command_msg_);
  ach_cancel(&motors_->cmd_chan, NULL);
  ach_close(&motors_->cmd_chan);
  ach_cancel(&motors_->state_chan, NULL);
  ach_close(&motors_->state_chan);
  somatic_motor_destroy(daemon_, motors_);
  delete motors_;
  delete waistd_command_channel_;
}

void WaistInterface::SendCommand(Somatic__WaistMode waist_mode) {
  somatic_waist_cmd_set(waistd_command_msg_, waist_mode);
  ach_status_t r = SOMATIC_PACK_SEND(waistd_command_channel_,
                                     somatic__waist_cmd, waistd_command_msg_);
  if (ACH_OK != r)
    fprintf(
        stderr, "[%s] Couldn't send message: %s\n", name_,
        ach_result_to_string(r));  // TODO: why stderr and not some somatic log?
}
void WaistInterface::Stop() { SendCommand(SOMATIC__WAIST_MODE__STOP); }

void WaistInterface::MoveForward() {
  SendCommand(SOMATIC__WAIST_MODE__MOVE_FWD);
}

void WaistInterface::MoveBackward() {
  SendCommand(SOMATIC__WAIST_MODE__MOVE_REV);
}

void WaistInterface::UpdateState() { somatic_motor_update(daemon_, motors_); }

std::vector<double> WaistInterface::GetPosition() {
  return std::vector<double>(motors_->pos, motors_->pos + 2);
}

std::vector<double> WaistInterface::GetVelocity() {
  return std::vector<double>(motors_->vel, motors_->vel + 2);
}

std::vector<double> WaistInterface::GetCurrent() {
  return std::vector<double>(motors_->cur, motors_->cur + 2);
}

boost::python::list WaistInterface::GetPositionExt() {
  return std_vector_to_py_list(GetPosition());
}

boost::python::list WaistInterface::GetVelocityExt() {
  return std_vector_to_py_list(GetVelocity());
}

boost::python::list WaistInterface::GetCurrentExt() {
  return std_vector_to_py_list(GetCurrent());
}

FloatingBaseStateSensorInterface::FloatingBaseStateSensorInterface(
    InterfaceContext& interface_context, std::string channel) {
  imu_chan_ = new ach_channel_t();
  daemon_ = &interface_context.daemon_;
  somatic_d_channel_open(daemon_, imu_chan_, channel.c_str(), NULL);
  UpdateState();
}

void FloatingBaseStateSensorInterface::UpdateState() {
  // Read data from ach channel
  static const int kImuChannelSize = 54;
  ach_status_t r;
  // This was originally done as I copied code from balancing
  // Turns out waiting is a bad idea if you can not
  // return to the control flow after KeyboardInterrupt (of Python)
  // to properly release the mutexes. Hence we changed WAIT to GET_LAST
  // in order to not wait
  /*
  struct timespec curr_time;
  clock_gettime(CLOCK_MONOTONIC, &curr_time);

  // Time till which we will wait for a new message
  static const double kWaitTime = 1.0 / 30.0;  // sec
  struct timespec abs_time =
      aa_tm_add(aa_tm_sec2timespec(kWaitTime), curr_time);

  // Sit and wait until the new message arrives or time is up
  imu_msg_ = SOMATIC_WAIT_LAST_UNPACK(r, somatic__vector, NULL, kImuChannelSize,
                                      imu_chan_, &abs_time);
  */
  imu_msg_ = SOMATIC_GET_LAST_UNPACK(r, somatic__vector, NULL, kImuChannelSize,
                                     imu_chan_);

  // If no new message, no need to update
  if (imu_msg_ == NULL) return;

  // Get the base angle and angular speed from the readings (note imu mounted
  // at 45 deg).
  static const double kMountAngle = -.7853981634;
  double new_x = imu_msg_->data[0] * cos(kMountAngle) -
                 imu_msg_->data[1] * sin(kMountAngle);
  base_angle_ = atan2(new_x, imu_msg_->data[2]);
  base_angular_speed_ = imu_msg_->data[3] * sin(kMountAngle) +
                        imu_msg_->data[4] * cos(kMountAngle);

  // Free the unpacked message
  somatic__vector__free_unpacked(imu_msg_, NULL);
}

void FloatingBaseStateSensorInterface::Destroy() {
  std::cout << "imu destroy" << std::endl;
  ach_cancel(imu_chan_, NULL);
  somatic_d_channel_close(daemon_, imu_chan_);
  delete imu_chan_;
}

WorldInterface::WorldInterface(InterfaceContext& interface_context,
                               std::string cmd_channel,
                               std::string state_channel, double max_wait_time)
    : max_wait_time_(max_wait_time) {
  daemon_ = &interface_context.daemon_;
  sim_command_msg_ = somatic_sim_cmd_alloc();
  sim_command_channel_ = new ach_channel_t();
  somatic_d_channel_open(daemon_, sim_command_channel_, cmd_channel.c_str(),
                         NULL);
  sim_state_channel_ = new ach_channel_t();
  somatic_d_channel_open(daemon_, sim_state_channel_, state_channel.c_str(),
                         NULL);
}

void WorldInterface::Destroy() {
  std::cout << "world interface destroy" << std::endl;
  somatic_sim_cmd_free(sim_command_msg_);
  ach_cancel(sim_command_channel_, NULL);
  somatic_d_channel_close(daemon_, sim_command_channel_);
  delete sim_command_channel_;
}

bool WorldInterface::Step() {
  somatic_sim_cmd_set(sim_command_msg_, SOMATIC__SIM_CMD__CODE__STEP, NULL);
  return SendCommand();
}

bool WorldInterface::Reset(struct Somatic_KrangPoseParams& pose) {
  somatic_sim_cmd_set(sim_command_msg_, SOMATIC__SIM_CMD__CODE__RESET, &pose);
  return SendCommand();
}

bool WorldInterface::ResetExt(boost::python::dict& pose_dict) {
  namespace py = boost::python;
  struct Somatic_KrangPoseParams pose;
  pose.heading = py::extract<double>(pose_dict["heading"]);
  pose.q_base = py::extract<double>(pose_dict["q_base"]);
  for (int i = 0; i < 3; i++) {
    py::tuple xyz_tuple = py::extract<py::tuple>(pose_dict["xyz"]);
    pose.xyz[i] = py::extract<double>(xyz_tuple[i]);
  }
  pose.q_lwheel = py::extract<double>(pose_dict["q_lwheel"]);
  pose.q_rwheel = py::extract<double>(pose_dict["q_rwheel"]);
  pose.q_waist = py::extract<double>(pose_dict["q_waist"]);
  pose.q_torso = py::extract<double>(pose_dict["q_torso"]);
  for (int i = 0; i < 7; i++) {
    py::tuple q_left_arm_tuple =
        py::extract<py::tuple>(pose_dict["q_left_arm"]);
    pose.q_left_arm[i] = py::extract<double>(q_left_arm_tuple[i]);
  }
  for (int i = 0; i < 7; i++) {
    py::tuple q_right_arm_tuple =
        py::extract<py::tuple>(pose_dict["q_right_arm"]);
    pose.q_right_arm[i] = py::extract<double>(q_right_arm_tuple[i]);
  }
  for (int i = 0; i < 2; i++) {
    py::tuple q_camera_tuple = py::extract<py::tuple>(pose_dict["q_camera"]);
    pose.q_camera[i] = py::extract<double>(q_camera_tuple[i]);
  }
  pose.init_with_balance_pose =
      py::extract<int>(pose_dict["init_with_balance_pose"]);
  return Reset(pose);
}

bool WorldInterface::SendCommand() {
  ach_flush(sim_state_channel_);
  ach_status_t rach = SOMATIC_PACK_SEND(sim_command_channel_, somatic__sim_cmd,
                                        sim_command_msg_);

  somatic_d_check(daemon_, SOMATIC__EVENT__PRIORITIES__CRIT,
                  SOMATIC__EVENT__CODES__COMM_FAILED_TRANSPORT, ACH_OK == rach,
                  "somatic_sim_cmd", "ach result: %s",
                  ach_result_to_string(rach));

  struct timespec curr_time;
  clock_gettime(CLOCK_MONOTONIC, &curr_time);
  struct timespec abs_time =
      aa_tm_add(aa_tm_sec2timespec(max_wait_time_), curr_time);
  Somatic__SimMsg* state = SOMATIC_WAIT_LAST_UNPACK(
      rach, somatic__sim_msg, NULL, 8 + 512, sim_state_channel_, &abs_time);

  if (state == NULL || rach == ACH_TIMEOUT) {
    std::cout
        << "Failure to receive acknowledgement within the specified wait time. "
           "Simulation program may not be alive. "
           "Or its operation time may be longer than the specified wait time."
        << std::endl;

    return false;
  }

  somatic__sim_msg__free_unpacked(state, NULL);

  return true;
}
