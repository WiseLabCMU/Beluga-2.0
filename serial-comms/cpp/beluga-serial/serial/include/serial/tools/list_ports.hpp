/**
 * @file list_ports.hpp
 *
 * @brief
 *
 * @date 1/29/25
 *
 * @author tom
 */

#ifndef BELUGA_FRAME_LIST_PORTS_HPP
#define BELUGA_FRAME_LIST_PORTS_HPP

#include <vector>

#if defined(__linux__)
#include <serial/tools/core/sys_fs_linux.hpp>
#else
#error "Not supported"
#endif

namespace SerialTools {
#if defined(__linux__)
using SysFS = SerialToolsInternal::SysFsLinux;
using SysFsScanAttr = SerialToolsInternal::SysFsLinuxScanAttr;
#endif

std::vector<SysFS> comports(const SysFsScanAttr &attr = SysFsScanAttr{});
} // namespace SerialTools

#endif // BELUGA_FRAME_LIST_PORTS_HPP
