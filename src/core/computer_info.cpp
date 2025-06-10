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

#include <filesystem>
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

CpuInfo::CpuInfo(void)
{
  // Load CPU information from /proc/cpuinfo
  std::ifstream cpuinfo_file("/proc/cpuinfo");
  if (!cpuinfo_file.is_open()) {
    throw std::runtime_error("Failed to open /proc/cpuinfo");
  }

  std::string line;
  while (std::getline(cpuinfo_file, line)) {
    if (line.find("processor") == 0) {
      ProcInfo proc_info;
      proc_info.idx = std::stoul(line.substr(line.find(':') + 1));
      proc_infos_.push_back(proc_info);
    } else if (line.find("model name") == 0) {
      proc_infos_.back().model = line.substr(line.find(':') + 2);
    } else if (line.find("cpu MHz") == 0) {
      proc_infos_.back().freq = std::stod(line.substr(line.find(':') + 1));
    } else if (line.find("core id") == 0) {
      proc_infos_.back().core_id = std::stoul(line.substr(line.find(':') + 1));
    }
  }

  cpuinfo_file.close();

  update();
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
  is_first_update_ = false;
}

void CpuInfo::updateTemp(void)
{
  // Load CPU temperature from /sys/class/thermal/thermal_zone*/temp
  const std::string thermal_path = "/sys/class/thermal/";
  for (const auto& entry : std::filesystem::directory_iterator(thermal_path)) {
    if (entry.is_directory() &&
        entry.path().string().find("thermal_zone") != std::string::npos) {
      std::ifstream type_file(entry.path() / "type");
      if (!type_file.is_open()) {
        continue;
      }
      std::string type;
      type_file >> type;
      type_file.close();
      if (type == "x86_pkg_temp" || type == "cpu-thermal" || type == "TCPU" ||
          type.find("cpu") != std::string::npos) {
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
