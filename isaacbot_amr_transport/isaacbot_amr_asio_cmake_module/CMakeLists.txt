# Copyright 2021 TierIV, Inc.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Modified 2026 by IsaacBot: renamed from asio_cmake_module to
# isaacbot_amr_asio_cmake_module as part of the isaacbot_amr_transport fork.

cmake_minimum_required(VERSION 3.14.4)

project(isaacbot_amr_asio_cmake_module)

find_package(ament_cmake REQUIRED)

ament_package(
  CONFIG_EXTRAS "isaacbot_amr_asio_cmake_module-extras.cmake"
)

install(DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME})
