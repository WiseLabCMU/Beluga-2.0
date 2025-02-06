/**
 * @file sys_fs_common.cpp
 *
 * @brief
 *
 * @date 1/21/25
 *
 * @author tom
 */

#include <serial/tools/core/sys_fs_common.hpp>
#include <sstream>

namespace SerialToolsInternal {

SysFsBase::SysFsBase(const fs::path &dev, bool skip_link_detection) {
    _device = dev.string();
    _name = dev.stem().string();
    _description = "n/a";
    _hwid = "n/a";
    _serial_number = "";
    _location = "";
    _manufacturer = "";
    _product = "";
    _interface = "";
    _vid = 0;
    _pid = 0;

    if (!skip_link_detection && !dev.empty() && is_symlink(dev)) {
        _hwid = "LINK=" + fs::canonical(dev).string();
    }
}

std::string SysFsBase::device() const noexcept { return _device; }

std::string SysFsBase::name() const noexcept { return _name; }

std::string SysFsBase::description() const noexcept { return _description; }

std::string SysFsBase::hwid() const noexcept { return _hwid; }

std::string SysFsBase::serial_number() const noexcept { return _serial_number; }

std::string SysFsBase::location() const noexcept { return _location; }

std::string SysFsBase::manufacturer() const noexcept { return _manufacturer; }

std::string SysFsBase::product() const noexcept { return _product; }

std::string SysFsBase::interface() const noexcept { return _interface; }

std::string SysFsBase::usb_description() {
    if (!_interface.empty()) {
        std::string description = _product + " - ";
        return description + _interface;
    } else if (!_product.empty()) {
        return _product;
    }
    return _name;
}

std::string SysFsBase::usb_info() {
    std::ostringstream oss;
    oss << "USB VID:PID=" << std::setw(4) << std::setfill('0') << std::uppercase
        << std::hex << _vid;
    oss << ":" << std::setw(4) << std::setfill('0') << std::uppercase
        << std::hex << _pid;
    oss << " SER=";
    if (!_serial_number.empty()) {
        oss << _serial_number;
    }
    oss << " LOCATION=";
    if (!_location.empty()) {
        oss << _location;
    }
    return oss.str();
}

void SysFsBase::_apply_usb_info() {
    _description = usb_description();
    _hwid = usb_info();
}
} // namespace SerialToolsInternal
