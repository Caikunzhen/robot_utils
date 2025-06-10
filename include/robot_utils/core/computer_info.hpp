/**
 *******************************************************************************
 * @file computer_info.hpp
 * @brief This file provides functions to get computer information
 *
 * @section history
 *
 * @version 1.0.0
 * @date 2025-06-10
 * @author Caikunzhen
 * @details
 * 1. Complete the computer_info.hpp
 *******************************************************************************
 *  Copyright (c) 2025 Caikunzhen, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ROBOT_UTILS_CORE_COMPUTER_INFO_HPP_
#define ROBOT_UTILS_CORE_COMPUTER_INFO_HPP_

/* Includes ------------------------------------------------------------------*/
#include <stdexcept>
#include <string>
#include <vector>
/* Exported macro ------------------------------------------------------------*/

namespace robot_utils
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

class CpuInfo
{
 public:
  struct ProcDynInfo {
    std::string name;      ///< CPU core name, e.g., "cpu", "cpu0", "cpu1", ...
    uint64_t user = 0;     ///< user time
    uint64_t nice = 0;     ///< nice time
    uint64_t system = 0;   ///< system time
    uint64_t idle = 0;     ///< idle time
    uint64_t iowait = 0;   ///< I/O wait time
    uint64_t irq = 0;      ///< hardware interrupt time
    uint64_t softirq = 0;  ///< software interrupt time
    uint64_t steal = 0;    ///< steal time
    uint64_t guest = 0;    ///< guest time
    uint64_t guest_nice = 0;  ///< guest nice time
    double usage = 0.0;       ///< CPU usage percentage, [0, 1]
  };

  struct ProcInfo {
    size_t idx = 0;      ///< index of processor
    std::string model;   ///< CPU model name
    double freq = 0.0;   ///< CPU frequency in MHz
    size_t core_id = 0;  ///< CPU core ID
  };

  CpuInfo(const CpuInfo&) = delete;
  CpuInfo& operator=(const CpuInfo&) = delete;
  CpuInfo(CpuInfo&&) = delete;
  CpuInfo& operator=(CpuInfo&&) = delete;

  /**
   * @brief Update CPU information
   * @note Call this function periodically to refresh the CPU information.
   */
  void update(void);

  /**
   * @brief Get information of all CPU cores
   * @return A vector of ProcInfo containing information of all CPU cores,
   * [cpu, cpu0, cpu1, ...]
   */
  const std::vector<ProcInfo>& getProcInfos(void) const { return proc_infos_; }

  /**
   * @brief Get dynamic information of all CPU cores
   * @return A vector of ProcDynInfo containing dynamic information of all CPU
   * cores, [cpu, cpu0, cpu1, ...]
   */
  const std::vector<ProcDynInfo>& getProcDynInfos(void) const
  {
    return proc_dyn_infos_;
  }

  size_t getNumCores(void) const { return proc_infos_.size(); }

  /**
   * @brief Get dynamic information of total CPU cores
   * @return Dynamic information of total CPU cores
   * @throw std::runtime_error if no CPU dynamic information is available
   */
  const ProcDynInfo& getProcDynInfo(void) const
  {
    if (proc_dyn_infos_.empty()) {
      throw std::runtime_error("No CPU dynamic information available");
    }
    return proc_dyn_infos_.front();
  }

  /**
   * @brief Get CPU temperature
   * @return CPU temperature in Celsius
   */
  double getTemp(void) const { return temp_; }

  static CpuInfo& GetInstance(void)
  {
    static CpuInfo instance;
    return instance;
  }

 private:
  CpuInfo(void);
  ~CpuInfo(void) = default;

  void updateDynInfo(void);

  void updateTemp(void);

  std::vector<ProcInfo> proc_infos_;  ///< information of all CPU cores
  /// dynamic information of all CPU cores
  std::vector<ProcDynInfo> proc_dyn_infos_;
  bool is_first_update_ = true;  ///< flag to indicate first update
  double temp_ = -1.0;           ///< CPU temperature in Celsius
};

class MemInfo
{
 public:
  MemInfo(const MemInfo&) = delete;
  MemInfo& operator=(const MemInfo&) = delete;
  MemInfo(MemInfo&&) = delete;
  MemInfo& operator=(MemInfo&&) = delete;

  /**
   * @brief Update memory information
   * @note Call this function periodically to refresh the memory information.
   */
  void update(void);

  /**
   * @brief Get total memory in kB
   * @return Total memory in kB
   */
  uint64_t getTotalMemory(void) const { return total_memory_; }

  /**
   * @brief Get available memory in kB
   * @return Available memory in kB
   */
  uint64_t getAvailableMemory(void) const { return available_memory_; }

  /**
   * @brief Get memory usage percentage
   * @return Memory usage percentage, [0, 1]
   * @throw std::runtime_error if total memory is zero
   */
  double getMemoryUsage(void) const
  {
    if (total_memory_ == 0) {
      throw std::runtime_error("Total memory is zero, cannot calculate usage");
    }
    return static_cast<double>(total_memory_ - available_memory_) /
           total_memory_;
  }

  /**
   * @brief Get total swap memory in kB
   * @return Total swap memory in kB
   */
  uint64_t getTotalSwap(void) const { return total_swap_; }

  /**
   * @brief Get free swap memory in kB
   * @return Free swap memory in kB
   */
  uint64_t getFreeSwap(void) const { return free_swap_; }

  /**
   * @brief Get swap memory usage percentage
   * @return Swap memory usage percentage, [0, 1]
   * @throw std::runtime_error if total swap memory is zero
   */
  double getSwapUsage(void) const
  {
    if (total_swap_ == 0) {
      throw std::runtime_error(
          "Total swap memory is zero, cannot calculate usage");
    }
    return static_cast<double>(total_memory_ - free_swap_) / total_swap_;
  }

  static MemInfo& GetInstance(void)
  {
    static MemInfo instance;
    return instance;
  }

 private:
  MemInfo(void) { update(); }
  ~MemInfo(void) = default;

  uint64_t total_memory_ = 0;      ///< total memory in kB
  uint64_t available_memory_ = 0;  ///< available memory in kB
  uint64_t total_swap_ = 0;        ///< total swap memory in kB
  uint64_t free_swap_ = 0;         ///< free swap memory in kB
};
}  // namespace robot_utils

#endif /*  ROBOT_UTILS_CORE_COMPUTER_INFO_HPP_*/
