/**
 * @file list_ports.cpp
 *
 * @brief
 *
 * @date 1/29/25
 *
 * @author tom
 */

#include <serial/tools/list_ports.hpp>
#include <vector>

namespace SerialTools {
std::vector<SysFS> comports(const SysFsScanAttr &attr) {
    return SerialToolsInternal::comports(attr);
}
} // namespace SerialTools
