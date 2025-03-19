/**
 * @file sys_fs_linux.hpp
 *
 * @brief
 *
 * @date 1/22/25
 *
 * @author tom
 */

#ifndef BELUGA_FRAME_SYS_FS_LINUX_HPP
#define BELUGA_FRAME_SYS_FS_LINUX_HPP

#include <serial/tools/core/sys_fs_common.hpp>
#include <vector>

namespace SerialToolsInternal {

struct SysFsLinuxScanAttr {
    bool ttyS = true;
    bool ttyUSB = true;
    bool ttyXRUSB = true;
    bool ttyACM = true;
    bool ttyAMA = true;
    bool rfcomm = true;
    bool ttyAP = true;
    bool ttyGS = true;
};

class SysFsLinux : public SysFsBase {
  public:
    explicit SysFsLinux(const fs::path &dev);

    [[nodiscard]] fs::path device_path() const noexcept;
    [[nodiscard]] fs::path usb_device_path() const noexcept;
    [[nodiscard]] std::string subsystem() const noexcept;
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

std::vector<SysFsLinux>
comports(const SysFsLinuxScanAttr &attr = SysFsLinuxScanAttr{});
} // namespace SerialToolsInternal

#endif // BELUGA_FRAME_SYS_FS_LINUX_HPP
