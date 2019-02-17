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
 * @file ach_interface.h
 * @author Munzir Zafar
 * @date Dec 4, 2018
 * @brief Ach interface for controlling krang
 */

#ifndef KRANG_CONTROL_ACH_INTERFACE_H_
#define KRANG_CONTROL_ACH_INTERFACE_H_

#include <somatic.h>  // has correct order of other includes

#include <ach.h>           // ach_channel_t
#include <somatic.pb-c.h>  // Somatic__Vector, Somatic__Waist: Mode, Cmd; Somatic__SimCmd
#include <somatic/daemon.h>  // somatic_d_t, somatic_d_opts_t
#include <somatic/motor.h>   // somatic_motor_t
#include <somatic/msg.h>     // Somatic_KrangPoseParams

#include <string>  // std::string
#include <vector>  // std::vector

#include <boost/python.hpp>

class InterfaceContext {
 public:
  InterfaceContext(const std::string daemon_identifier);
  ~InterfaceContext() { Destroy(); }
  void Run();
  void Destroy();
  somatic_d_t daemon_;
  somatic_d_opts_t daemon_opts_;
};

class MotorInterface {
 public:
  MotorInterface(InterfaceContext& interface_context, std::string name,
                 std::string command_channel_name,
                 std::string state_channel_name, int num);
  ~MotorInterface() { Destroy(); }
  void Destroy();

  void PositionCommand(const std::vector<double>& val);
  void VelocityCommand(const std::vector<double>& val);
  void CurrentCommand(const std::vector<double>& val);
  void PositionCommandExt(const boost::python::list& val);
  void VelocityCommandExt(const boost::python::list& val);
  void CurrentCommandExt(const boost::python::list& val);
  void LockCommand();
  void UnlockCommand();
  void UpdateState();
  std::vector<double> GetPosition();
  std::vector<double> GetVelocity();
  std::vector<double> GetCurrent();
  boost::python::list GetPositionExt();
  boost::python::list GetVelocityExt();
  boost::python::list GetCurrentExt();

  char name_[128];
  somatic_d_t* daemon_;
  size_t n_;
  somatic_motor_t* motors_;
};

class WaistInterface {
 public:
  WaistInterface(InterfaceContext& interface_context, std::string name,
                 std::string command_channel_name,
                 std::string state_channel_name);
  ~WaistInterface() { Destroy(); }
  void Destroy();

  void Stop();
  void MoveForward();
  void MoveBackward();
  void UpdateState();
  std::vector<double> GetPosition();
  std::vector<double> GetVelocity();
  std::vector<double> GetCurrent();
  boost::python::list GetPositionExt();
  boost::python::list GetVelocityExt();
  boost::python::list GetCurrentExt();

 private:
  void SendCommand(Somatic__WaistMode waist_mode);
  char name_[128];
  somatic_d_t* daemon_;
  ach_channel_t* waistd_command_channel_;
  somatic_motor_t* motors_;
  Somatic__WaistCmd* waistd_command_msg_;
};

class FloatingBaseStateSensorInterface {
 public:
  FloatingBaseStateSensorInterface(InterfaceContext& interface_context,
                                   std::string channel);
  ~FloatingBaseStateSensorInterface() { Destroy(); }
  void UpdateState();
  void Destroy();
  double GetBaseAngle() const { return base_angle_; }
  double GetBaseAngularSpeed() const { return base_angular_speed_; }

  ach_channel_t* imu_chan_;   // unused
  Somatic__Vector* imu_msg_;  // unused
  somatic_d_t* daemon_;       // unused
  double base_angle_;
  double base_angular_speed_;
};

class WorldInterface {
 public:
  WorldInterface(InterfaceContext& interface_context, std::string cmd_channel,
                 std::string state_channel, double max_wait_time = 5.0 /*sec*/);
  ~WorldInterface() { Destroy(); }
  void Destroy();
  bool Step();
  bool Reset(struct Somatic_KrangPoseParams& pose);
  bool ResetExt(boost::python::dict& pose_dict);

 private:
  bool SendCommand();
  somatic_d_t* daemon_;
  ach_channel_t* sim_command_channel_;
  ach_channel_t* sim_state_channel_;
  Somatic__SimCmd* sim_command_msg_;
  Somatic__SimMsg* sim_state_msg_;
  double max_wait_time_;
};
#endif  // KRANG_CONTROL_ACH_INTERFACE_H_
