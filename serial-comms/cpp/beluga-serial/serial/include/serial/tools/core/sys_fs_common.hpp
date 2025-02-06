/**
 * @file sys_fs_common.hpp
 *
 * @brief
 *
 * @date 1/21/25
 *
 * @author tom
 */

#ifndef BELUGA_FRAME_SYS_FS_COMMON_HPP
#define BELUGA_FRAME_SYS_FS_COMMON_HPP

#include <filesystem>
#include <string>

namespace SerialToolsInternal {
namespace fs = std::filesystem;
class SysFsBase {
  public:
    explicit SysFsBase(const fs::path &dev, bool skip_link_detection = false);

    [[nodiscard]] std::string device() const noexcept;
    [[nodiscard]] std::string name() const noexcept;
    [[nodiscard]] std::string description() const noexcept;
    [[nodiscard]] std::string hwid() const noexcept;
    [[nodiscard]] std::string serial_number() const noexcept;
    [[nodiscard]] std::string location() const noexcept;
    [[nodiscard]] std::string manufacturer() const noexcept;
    [[nodiscard]] std::string product() const noexcept;
    [[nodiscard]] std::string interface() const noexcept;

    std::string usb_description();
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
