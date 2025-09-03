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
    device_ = dev.string();
    name_ = dev.stem().string();
    description_ = "n/a";
    hwid_ = "n/a";
    serial_number_ = "";
    location_ = "";
    manufacturer_ = "";
    product_ = "";
    interface_ = "";
    vid_ = 0;
    pid_ = 0;

    if (!skip_link_detection && !dev.empty() && is_symlink(dev)) {
        hwid_ = "LINK=" + fs::canonical(dev).string();
    }
}

std::string SysFsBase::device() const noexcept { return device_; }

std::string SysFsBase::name() const noexcept { return name_; }

std::string SysFsBase::description() const noexcept { return description_; }

std::string SysFsBase::hwid() const noexcept { return hwid_; }

std::string SysFsBase::serial_number() const noexcept { return serial_number_; }

std::string SysFsBase::location() const noexcept { return location_; }

std::string SysFsBase::manufacturer() const noexcept { return manufacturer_; }

std::string SysFsBase::product() const noexcept { return product_; }

std::string SysFsBase::interface() const noexcept { return interface_; }

std::string SysFsBase::usb_description() {
    if (!interface_.empty()) {
        std::string description = product_ + " - ";
        return description + interface_;
    } else if (!product_.empty()) {
        return product_;
    }
    return name_;
}

std::string SysFsBase::usb_info() {
    std::ostringstream oss;
    oss << "USB VID:PID=" << std::setw(4) << std::setfill('0') << std::uppercase
        << std::hex << vid_;
    oss << ":" << std::setw(4) << std::setfill('0') << std::uppercase
        << std::hex << pid_;
    oss << " SER=";
    if (!serial_number_.empty()) {
        oss << serial_number_;
    }
    oss << " LOCATION=";
    if (!location_.empty()) {
        oss << location_;
    }
    return oss.str();
}

void SysFsBase::apply_usb_info_() {
    description_ = usb_description();
    hwid_ = usb_info();
}
} // namespace SerialToolsInternal
