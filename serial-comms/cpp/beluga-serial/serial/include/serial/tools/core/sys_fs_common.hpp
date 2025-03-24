/**
 * @file sys_fs_common.hpp
 *
 * @brief Helper module for the various platform dependent comport
 * implementations.
 *
 * @date 1/21/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#ifndef BELUGA_FRAME_SYS_FS_COMMON_HPP
#define BELUGA_FRAME_SYS_FS_COMMON_HPP

#include <filesystem>
#include <string>

namespace SerialToolsInternal {
namespace fs = std::filesystem;

/// Info collection base class for serial ports
class SysFsBase {
  public:
    /**
     * Instantiates the attributes of the SysFsBase class
     * @param[in] dev Path to the device
     * @param[in] skip_link_detection Flag to indicate whether links should be
     * handled differently
     */
    explicit SysFsBase(const fs::path &dev, bool skip_link_detection = false);

    /**
     * Device path
     */
    [[nodiscard]] std::string device() const noexcept;

    /**
     * Device name
     */
    [[nodiscard]] std::string name() const noexcept;

    /**
     * Device description
     */
    [[nodiscard]] std::string description() const noexcept;

    /**
     * Device hardware ID
     */
    [[nodiscard]] std::string hwid() const noexcept;

    /**
     * Device serial number
     */
    [[nodiscard]] std::string serial_number() const noexcept;

    /**
     * Device location
     */
    [[nodiscard]] std::string location() const noexcept;

    /**
     * Device manufacturer
     */
    [[nodiscard]] std::string manufacturer() const noexcept;

    /**
     * Device product name
     */
    [[nodiscard]] std::string product() const noexcept;

    /**
     * Device interface
     */
    [[nodiscard]] std::string interface() const noexcept;

    /**
     * Short string to name the port based on USB info
     * @return The USB info string
     */
    std::string usb_description();

    /**
     * String with USB relevant information about the device
     * @return The USB device info string
     */
    std::string usb_info();

  protected:
    std::string _device;
    std::string _name;
    std::string _description;
    std::string _hwid;
    std::string _serial_number;
    std::string _location;
    std::string _manufacturer;
    std::string _product;
    std::string _interface;
    uint64_t _vid;
    uint64_t _pid;

    void _apply_usb_info();
};
} // namespace SerialToolsInternal

#endif // BELUGA_FRAME_SYS_FS_COMMON_HPP
