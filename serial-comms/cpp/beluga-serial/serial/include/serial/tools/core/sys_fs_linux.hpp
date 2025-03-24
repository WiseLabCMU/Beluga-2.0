/**
 * @file sys_fs_linux.hpp
 *
 * @brief This is a module that gathers a list of serial ports including details
 * on GNU/Linux systems.
 *
 * @date 1/22/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#ifndef BELUGA_FRAME_SYS_FS_LINUX_HPP
#define BELUGA_FRAME_SYS_FS_LINUX_HPP

#include <serial/tools/core/sys_fs_common.hpp>
#include <vector>

namespace SerialToolsInternal {

/**
 * Port scanning attributes for Linux
 */
struct SysFsLinuxScanAttr {
    bool ttyS = true;     ///< Scan for ttyS* devices
    bool ttyUSB = true;   ///< Scan for ttyUSB* devices
    bool ttyXRUSB = true; ///< Scan for ttyXRUSB* devices
    bool ttyACM = true;   ///< Scan for ttyACM* devices
    bool ttyAMA = true;   ///< Scan for ttyAMA* devices
    bool rfcomm = true;   ///< Scan for rfcomm* devices
    bool ttyAP = true;    ///< Scan for ttyAP* devices
    bool ttyGS = true;    ///< Scan for ttyGS* devices
};

/// Wrapper for easy sysfs access and device info
class SysFsLinux : public SysFsBase {
  public:
    /**
     * Constructor for SysFsLinux objects
     * @param[in] dev The path to the device
     */
    explicit SysFsLinux(const fs::path &dev);

    /**
     * The real path to the device
     * @return The real path
     */
    [[nodiscard]] fs::path device_path() const noexcept;

    /**
     * The real path to the USB device
     * @return The real path
     */
    [[nodiscard]] fs::path usb_device_path() const noexcept;

    /**
     * The serial subsystem
     * @return The subsystem
     */
    [[nodiscard]] std::string subsystem() const noexcept;

    /**
     * The USB interface path
     * @return interface path
     */
    [[nodiscard]] fs::path usb_interface_path() const noexcept;

  private:
    fs::path _device_path;
    fs::path _usb_device_path;
    std::string _subsystem;
    fs::path _usb_interface_path;

    void _fill_usb_dev_info();

    static std::string _read_line(const fs::path &path,
                                  const std::string &file);
};

/**
 * Gathers a list of available serial ports on Linux
 * @param[in] attr The ports to scan for. If no attributes are specified, then
 * all ports are scanned.
 * @return A list of serial device paths
 */
std::vector<SysFsLinux>
comports(const SysFsLinuxScanAttr &attr = SysFsLinuxScanAttr{});
} // namespace SerialToolsInternal

#endif // BELUGA_FRAME_SYS_FS_LINUX_HPP
