/**
 *******************************************************************************
 * @file computer_info.cpp
 * @brief This file provides functions to get computer information
 *
 * @section history
 *
 * @version 1.0.0
 * @date 2025-06-10
 * @author Caikunzhen
 * @details
 * 1. Complete the computer_info.cpp
 *******************************************************************************
 *  Copyright (c) 2025 Caikunzhen, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "robot_utils/core/computer_info.hpp"

#include <experimental/filesystem>
#include <fstream>
#include <sstream>
/* Private macro -------------------------------------------------------------*/

namespace robot_utils
{
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

void CpuInfo::update(void)
{
  updateDynInfo();

  updateTemp();
}

void CpuInfo::updateDynInfo(void)
{
  // Load dynamic CPU information from /proc/stat
  std::ifstream stat_file("/proc/stat");
  if (!stat_file.is_open()) {
    throw std::runtime_error("Failed to open /proc/stat");
  }
  std::string line;
  size_t idx = 0;
  while (std::getline(stat_file, line)) {
    if (line.find("cpu") == 0) {
      ProcDynInfo proc_dyn_info;
      proc_dyn_info.name = line.substr(0, line.find(' '));
      std::istringstream iss(line.substr(line.find(' ') + 1));
      iss >> proc_dyn_info.user >> proc_dyn_info.nice >> proc_dyn_info.system >>
          proc_dyn_info.idle >> proc_dyn_info.iowait >> proc_dyn_info.irq >>
          proc_dyn_info.softirq >> proc_dyn_info.steal >> proc_dyn_info.guest >>
          proc_dyn_info.guest_nice;

      if (!is_first_update_) {
        // Calculate CPU usage
        uint64_t total_time = proc_dyn_info.user + proc_dyn_info.nice +
                              proc_dyn_info.system + proc_dyn_info.idle +
                              proc_dyn_info.iowait + proc_dyn_info.irq +
                              proc_dyn_info.softirq + proc_dyn_info.steal +
                              proc_dyn_info.guest + proc_dyn_info.guest_nice;
        uint64_t active_time =
            total_time - proc_dyn_info.idle - proc_dyn_info.iowait;
        uint64_t prev_total_time =
            proc_dyn_infos_[idx].user + proc_dyn_infos_[idx].nice +
            proc_dyn_infos_[idx].system + proc_dyn_infos_[idx].idle +
            proc_dyn_infos_[idx].iowait + proc_dyn_infos_[idx].irq +
            proc_dyn_infos_[idx].softirq + proc_dyn_infos_[idx].steal +
            proc_dyn_infos_[idx].guest + proc_dyn_infos_[idx].guest_nice;
        uint64_t prev_active_time = prev_total_time -
                                    proc_dyn_infos_[idx].idle -
                                    proc_dyn_infos_[idx].iowait;
        uint64_t total_diff = total_time - prev_total_time;
        uint64_t active_diff = active_time - prev_active_time;
        if (total_diff > 0) {
          proc_dyn_info.usage = static_cast<double>(active_diff) / total_diff;
        } else {  // Avoid division by zero
          proc_dyn_info.usage = 0.0;
        }
        proc_dyn_infos_[idx] = proc_dyn_info;  // Update existing info
      } else {
        proc_dyn_info.usage = 0.0;
        proc_dyn_infos_.push_back(proc_dyn_info);
      }

      ++idx;
    }
  }

  stat_file.close();

  // Load CPU frequency from
  // /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq
  proc_dyn_infos_[0].freq = 0.0;
  for (size_t i = 1; i < proc_dyn_infos_.size(); ++i) {
    const std::string freq_path = "/sys/devices/system/cpu/" +
                                  proc_dyn_infos_[i].name +
                                  "/cpufreq/scaling_cur_freq";
    std::ifstream freq_file(freq_path);
    if (freq_file.is_open()) {
      double freq;
      freq_file >> freq;
      freq_file.close();
      proc_dyn_infos_[i].freq = freq / 1000.0;  // Convert from kHz to MHz
    } else {
      proc_dyn_infos_[i].freq = 0.0;  // Set frequency to 0 if not available
    }
    proc_dyn_infos_[0].freq += proc_dyn_infos_[i].freq;
  }
  proc_dyn_infos_[0].freq /= (proc_dyn_infos_.size() - 1);  // Average frequency

  is_first_update_ = false;
}

void CpuInfo::updateTemp(void)
{
  namespace fs = std::experimental::filesystem;
  // Load CPU temperature from /sys/class/thermal/thermal_zone*/temp
  const std::string thermal_path = "/sys/class/thermal/";
  for (const auto& entry : fs::directory_iterator(thermal_path)) {
    if (fs::is_regular_file(entry.status()) &&
        entry.path().string().find("thermal_zone") != std::string::npos) {
      std::ifstream type_file(entry.path() / "type");
      if (!type_file.is_open()) {
        continue;
      }
      std::string type;
      type_file >> type;
      type_file.close();
      if (type == "x86_pkg_temp" || type == "cpu-thermal" || type == "TCPU" ||
          type.find("cpu") != std::string::npos ||
          type.find("CPU") != std::string::npos) {
        std::ifstream temp_file(entry.path() / "temp");
        if (!temp_file.is_open()) {
          continue;
        }
        double temp;
        temp_file >> temp;
        temp_file.close();
        temp_ = temp / 1000.0;  // Convert from millidegree Celsius to Celsius
        return;
      }
    }
  }
}

void MemInfo::update(void)
{
  // Load memory information from /proc/meminfo
  std::ifstream meminfo_file("/proc/meminfo");
  if (!meminfo_file.is_open()) {
    throw std::runtime_error("Failed to open /proc/meminfo");
  }

  std::string line;
  while (std::getline(meminfo_file, line)) {
    std::istringstream iss(line);
    std::string key;
    iss >> key;

    if (key == "MemTotal:") {
      iss >> total_memory_;
    } else if (key == "MemAvailable:") {
      iss >> available_memory_;
    } else if (key == "SwapTotal:") {
      iss >> total_swap_;
    } else if (key == "SwapFree:") {
      iss >> free_swap_;
    }
  }

  meminfo_file.close();
}
/* Private function definitions ----------------------------------------------*/
}  // namespace robot_utils
