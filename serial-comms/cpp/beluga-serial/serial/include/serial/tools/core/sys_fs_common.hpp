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

#ifndef BELUGA_SERIAL_SYS_FS_COMMON_HPP
#define BELUGA_SERIAL_SYS_FS_COMMON_HPP

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
    std::string device_;
    std::string name_;
    std::string description_;
    std::string hwid_;
    std::string serial_number_;
    std::string location_;
    std::string manufacturer_;
    std::string product_;
    std::string interface_;
    uint64_t vid_;
    uint64_t pid_;

    void apply_usb_info_();
};
} // namespace SerialToolsInternal

#endif // BELUGA_SERIAL_SYS_FS_COMMON_HPP
