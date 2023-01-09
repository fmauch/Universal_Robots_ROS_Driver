// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
// Created on behalf of Universal Robots A/S
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2019-04-11
 *
 */
//----------------------------------------------------------------------
#include <pthread.h>
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include <chrono>
#include <csignal>
#include <cstddef>
#include <thread>
#include <numeric>
#include <ur_robot_driver/hardware_interface.h>
#include <ur_robot_driver/urcl_log_handler.h>

std::unique_ptr<ur_driver::HardwareInterface> g_hw_interface;

void signalHandler(int signum)
{
  std::cout << "Interrupt signal (" << signum << ") received.\n";

  g_hw_interface.reset();
  // cleanup and close up stuff here
  // terminate program

  exit(signum);
}

ros::Duration average(std::vector<ros::Duration> const& v)
{
  if (v.empty())
  {
    return ros::Duration();
  }

  auto const count = 1.0 / static_cast<double>(v.size());
  return std::reduce(v.begin(), v.end()) * count;
}

bool hasRealtimeKernel()
{
  std::ifstream realtime_file("/sys/kernel/realtime", std::ios::in);
  bool has_realtime = false;
  if (realtime_file.is_open())
  {
    realtime_file >> has_realtime;
  }
  else
  {
    ROS_ERROR_STREAM("Could not read '/sys/kernel/realtime'. This probably is no standard ubuntu system. Scheduling "
                     "might still be possible, but we cannot check for the kernel's realtime support and assume it does not..");
    return false;
  }
  return has_realtime;
}

bool setFiFoScheduling(pthread_t& thread, const int priority)
{
  struct sched_param params;
  params.sched_priority = priority;
  int ret = pthread_setschedparam(thread, SCHED_FIFO, &params);
  if (ret != 0)
  {
    ROS_ERROR_STREAM("Unsuccessful in setting thread to FIFO scheduling with priority " << priority << ". "
                                                                                        << strerror(ret));
    // TODO: Catch error code 1 (no permission) and print separate information
  }
  // Now verify the change in thread priority
  int policy = 0;
  ret = pthread_getschedparam(thread, &policy, &params);
  if (ret != 0)
  {
    ROS_ERROR("Couldn't retrieve real-time scheduling parameters");
    return false;
  }

  // Check the correct policy was applied
  if (policy != SCHED_FIFO)
  {
    ROS_ERROR("Scheduling is NOT SCHED_FIFO!");
    return false;
  }
  else
  {
    ROS_INFO_STREAM("SCHED_FIFO OK, priority " << params.sched_priority);
    if (params.sched_priority != priority)
    {
      return false;
    }
  }
  return true;
}

bool setCpuAffinity(pthread_t& thread, const int cpu_core)
{
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(cpu_core, &cpuset);
  int rc = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
  if (rc != 0)
  {
    ROS_ERROR_STREAM("Error setting thread affinity to " << rc);
    return false;
  }
  ROS_INFO_STREAM("Set cpu affinity to core " << cpu_core);
  return true;
}

int main(int argc, char** argv)
{
  // Set up ROS.
  ros::init(argc, argv, "ur_hardware_interface");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // register signal SIGINT and signal handler
  signal(SIGINT, signalHandler);

  ur_driver::registerUrclLogHandler();

  bool do_fifo_scheduling = nh_priv.param("do_fifo_scheduling", true);
  int cpu_affinity = nh_priv.param("cpu_affinity", -1);
  bool non_blocking_read = nh_priv.param("non_blocking_read", false);

  bool has_realtime = hasRealtimeKernel();
  ROS_INFO_STREAM("This system has " << (has_realtime ? "a" : "no") << " real-time kernel");

  pthread_t this_thread = pthread_self();

  if (do_fifo_scheduling)
  {
    const int max_thread_priority = sched_get_priority_max(SCHED_FIFO);
    ROS_INFO_STREAM("Max thread scheduling priority is " << max_thread_priority);
    if (max_thread_priority != -1)
    {
      setFiFoScheduling(this_thread, max_thread_priority);
    }
    else
    {
      ROS_ERROR("Could not get maximum thread priority for main thread");
    }
  }
  else {
    ROS_INFO("No FIFO scheduling requested");
  }

  if (cpu_affinity > 0)
  {
    setCpuAffinity(this_thread, cpu_affinity);
  }

  // Set up timers
  ros::Time timestamp;
  ros::Duration period;
  auto stopwatch_last = std::chrono::high_resolution_clock::now();
  auto stopwatch_now = stopwatch_last;

  g_hw_interface.reset(new ur_driver::HardwareInterface);

  if (!g_hw_interface->init(nh, nh_priv))
  {
    ROS_ERROR_STREAM("Could not correctly initialize robot. Exiting");
    exit(1);
  }
  ROS_DEBUG_STREAM("initialized hw interface");
  controller_manager::ControllerManager cm(g_hw_interface.get(), nh);

  // Get current time and elapsed time since last read
  timestamp = ros::Time::now();
  stopwatch_now = std::chrono::high_resolution_clock::now();
  period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
  stopwatch_last = stopwatch_now;

  double expected_cycle_time = 1.0 / (static_cast<double>(g_hw_interface->getControlFrequency()));

  const size_t MAX_CYCLES = 100000;
  std::vector<ros::Duration> timings(MAX_CYCLES, ros::Duration());
  size_t cycles_done = 0;

  // Run as fast as possible
  while (ros::ok() && cycles_done < MAX_CYCLES)
  {
    // Receive current state from robot
    g_hw_interface->read(timestamp, period);

    // Get current time and elapsed time since last read
    timestamp = ros::Time::now();
    stopwatch_now = std::chrono::high_resolution_clock::now();
    period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
    stopwatch_last = stopwatch_now;

    cm.update(timestamp, period, g_hw_interface->shouldResetControllers());

    g_hw_interface->write(timestamp, period);
    timings[cycles_done] = period;
    auto diff = expected_cycle_time - period.toSec();
    if (non_blocking_read)
    {
      //std::cout << "period: " << period << std::endl;
      std::chrono::duration sleep_period = std::chrono::duration<double, std::ratio<1>>(diff);
      //std::cout << "sleeping for " << sleep_period.count() << " seconds." << std::endl;
      std::this_thread::sleep_for(sleep_period);
    }
;
    cycles_done++;
  }

  spinner.stop();
  ROS_INFO_STREAM_NAMED("hardware_interface", "Shutting down.");

  std::ofstream outFile("/tmp/timings.txt");
  // the important part
  for (const auto& e : timings)
    outFile << e.toNSec() << "\n";

  ROS_INFO_STREAM("average cycle time: " << average(timings));
  ROS_INFO_STREAM("max cycle time: " << *std::max_element(timings.begin(), timings.end()));
  ROS_INFO_STREAM("min cycle time: " << *std::min_element(timings.begin(), timings.end()));

  return 0;
}
