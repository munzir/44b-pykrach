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
 * @file krach_ext.cpp
 * @author Munzir Zafar
 * @date-created Dec 5, 2018
 * @date-modified Feb 17, 2018
 * @brief Python module to interface with krang simulation
 */

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <krach/krach.h>

BOOST_PYTHON_MODULE(pykrach) {
  namespace py = boost::python;

  py::class_<std::vector<double>>("VectorOfDoubles")
      .def(py::vector_indexing_suite<std::vector<double>>());

  py::class_<InterfaceContext>("InterfaceContext",
                               py::init<const std::string>())
      .def("Run", &InterfaceContext::Run)
      .def("Destroy", &InterfaceContext::Destroy);

  py::class_<MotorInterface>(
      "MotorInterface",
      py::init<InterfaceContext&, std::string, std::string, std::string, int>())
      .def("Destroy", &MotorInterface::Destroy)
      .def("PositionCommand", &MotorInterface::PositionCommandExt)
      .def("VelocityCommand", &MotorInterface::VelocityCommandExt)
      .def("CurrentCommand", &MotorInterface::CurrentCommandExt)
      .def("LockCommand", &MotorInterface::LockCommand)
      .def("UnlockCommand", &MotorInterface::UnlockCommand)
      .def("UpdateState", &MotorInterface::UpdateState)
      .def("GetPosition", &MotorInterface::GetPositionExt)
      .def("GetVelocity", &MotorInterface::GetVelocityExt)
      .def("GetCurrent", &MotorInterface::GetCurrentExt);

  py::class_<WaistInterface>(
      "WaistInterface",
      py::init<InterfaceContext&, std::string, std::string, std::string>())
      .def("Destroy", &WaistInterface::Destroy)
      .def("Stop", &WaistInterface::Stop)
      .def("MoveForward", &WaistInterface::MoveForward)
      .def("MoveBackward", &WaistInterface::MoveBackward)
      .def("UpdateState", &WaistInterface::UpdateState)
      .def("GetPosition", &WaistInterface::GetPositionExt)
      .def("GetVelocity", &WaistInterface::GetVelocityExt)
      .def("GetCurrent", &WaistInterface::GetCurrentExt);

  py::class_<FloatingBaseStateSensorInterface>(
      "FloatingBaseStateSensorInterface",
      py::init<InterfaceContext&, std::string>())
      .def("UpdateState", &FloatingBaseStateSensorInterface::UpdateState)
      .def("Destroy", &FloatingBaseStateSensorInterface::Destroy)
      .add_property("base_angle",
                    &FloatingBaseStateSensorInterface::GetBaseAngle)
      .add_property("base_angular_speed",
                    &FloatingBaseStateSensorInterface::GetBaseAngularSpeed);

  py::class_<WorldInterface>(
      "WorldInterface", py::init<InterfaceContext&, std::string, std::string,
                                 py::optional<double>>())
      .def("Destroy", &WorldInterface::Destroy)
      .def("Step", &WorldInterface::Step)
      .def("Reset", &WorldInterface::ResetExt);
}

