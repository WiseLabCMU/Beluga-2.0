/**
 * @file list_ports.hpp
 *
 * @brief Serial port enumeration. This module will select the appropriate
 * backend and provides a function called `comports`. `comports` will return a
 * list of available serial ports on the system. Note that on some systems,
 * non-existent ports may be included in the list.
 *
 * @note Only Linux is supported at this time.
 *
 * @date 1/29/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
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

/**
 * Gathers a list of available serial ports on Linux
 * @param[in] attr The ports to scan for. If no attributes are specified, then
 * all ports are scanned.
 * @return A list of serial device paths
 */
std::vector<SysFS> comports(const SysFsScanAttr &attr = SysFsScanAttr{});
} // namespace SerialTools

#endif // BELUGA_FRAME_LIST_PORTS_HPP
