/**
 * @file sys_fs_linux.cpp
 *
 * @brief
 *
 * @date 1/22/25
 *
 * @author tom
 */

#include <fstream>
#include <glob.h>
#include <serial/tools/core/sys_fs_linux.hpp>

namespace SerialToolsInternal {
SysFsLinux::SysFsLinux(const fs::path &dev) : SysFsBase(dev) {
    fs::path device, device_path;
    bool is_link = false;

    if (!dev.empty() && fs::is_symlink(dev)) {
        device = fs::read_symlink(dev);
        is_link = true;
    } else {
        device = dev;
    }

    _usb_device_path = fs::path();
    device_path = fs::path("/sys/class/tty") / _name / "device";
    if (!device_path.empty()) {
        _device_path = fs::canonical(device_path);
        _subsystem = fs::canonical(fs::path(_device_path) / "subsystem")
                         .filename()
                         .string();
    } else {
        _device_path = fs::path();
        _subsystem = "";
    }

    if (_subsystem == "usb-serial") {
        _usb_interface_path = device_path.parent_path();
    } else if (_subsystem == "usb") {
        _usb_interface_path = _device_path;
    } else {
        _usb_interface_path = fs::path();
    }

    if (!_usb_interface_path.empty()) {
        _fill_usb_dev_info();
    }

    if (_subsystem == "usb" || _subsystem == "usb-serial") {
        _apply_usb_info();
    } else if (_subsystem == "pnp") {
        _description = _name;
        _hwid = SysFsLinux::_read_line(_device_path, "id");
    } else if (_subsystem == "amba") {
        _description = _name;
        _hwid = _device_path.stem().string();
    }

    if (is_link) {
        std::string link = " LINK=" + device.string();
        _hwid += link;
    }
}

std::string SysFsLinux::subsystem() const noexcept { return _subsystem; }

void SysFsLinux::_fill_usb_dev_info() {
    std::string line;
    uint32_t num_interfaces;
    _usb_device_path = _usb_interface_path.parent_path();
    line = SysFsLinux::_read_line(_usb_device_path, "bNumInterfaces");
    if (line.empty()) {
        num_interfaces = 1;
    } else {
        num_interfaces = std::stoul(line);
    }

    line = SysFsLinux::_read_line(_usb_device_path, "idVendor");
    _vid = std::stoull(line, nullptr, 16);
    line = SysFsLinux::_read_line(_usb_device_path, "idProduct");
    _pid = std::stoull(line, nullptr, 16);
    _serial_number = SysFsLinux::_read_line(_usb_device_path, "serial");
    if (num_interfaces > 1) {
        _location = _usb_interface_path.stem().string();
    } else {
        _location = _usb_device_path.stem().string();
    }
    _manufacturer = SysFsLinux::_read_line(_usb_device_path, "manufacturer");
    _product = SysFsLinux::_read_line(_usb_device_path, "product");
    _interface = SysFsLinux::_read_line(_usb_interface_path, "interface");
}

std::string SysFsLinux::_read_line(const fs::path &path,
                                   const std::string &file) {
    fs::path filepath = path / file;
    std::ifstream file_(filepath);
    std::string line;

    if (!file_.is_open()) {
        return "";
    }

    if (std::getline(file_, line)) {
        line.erase(0, line.find_first_not_of(" \t\n\r\f\v"));
        line.erase(line.find_last_not_of(" \t\n\r\f\v") + 1);
    }
    return line;
}

fs::path SysFsLinux::device_path() const noexcept { return _device_path; }

fs::path SysFsLinux::usb_device_path() const noexcept {
    return _usb_device_path;
}

fs::path SysFsLinux::usb_interface_path() const noexcept {
    return _usb_interface_path;
}

static std::vector<std::string> _glob(const char *pattern) {
    glob_t glob_result;
    std::vector<std::string> results;

    if (glob(pattern, 0, NULL, &glob_result) == 0) {
        for (size_t i = 0; i < glob_result.gl_pathc; i++) {
            results.push_back(std::string(glob_result.gl_pathv[i]));
        }
        globfree(&glob_result);
    }

    return results;
}

#define APPEND_VECTOR(_destination, _source)                                   \
    (_destination)                                                             \
        .insert((_destination).end(), (_source).begin(), (_source).end())
#define UPDATE_VECTOR(attr, _destination, _source, _pattern)                   \
    do {                                                                       \
        if (attr) {                                                            \
            _source = _glob(_pattern);                                         \
            APPEND_VECTOR(_destination, _source);                              \
        }                                                                      \
    } while (false)

std::vector<SysFsLinux> comports(const SysFsLinuxScanAttr &attr) {
    std::vector<std::string> devices, results;
    std::vector<SysFsLinux> ret;

    UPDATE_VECTOR(attr.ttyS, devices, results, "/dev/ttyS*");
    UPDATE_VECTOR(attr.ttyUSB, devices, results, "/dev/ttyUSB*");
    UPDATE_VECTOR(attr.ttyXRUSB, devices, results, "/dev/ttyXRUSB*");
    UPDATE_VECTOR(attr.ttyACM, devices, results, "/dev/ttyACM**");
    UPDATE_VECTOR(attr.ttyAMA, devices, results, "/dev/ttyAMA*");
    UPDATE_VECTOR(attr.rfcomm, devices, results, "/dev/rfcomm*");
    UPDATE_VECTOR(attr.ttyAP, devices, results, "/dev/ttyAP*");
    UPDATE_VECTOR(attr.ttyGS, devices, results, "/dev/ttyGS*");

    for (size_t i = 0; i < devices.size(); i++) {
        SysFsLinux dev(devices[i]);
        if (dev.subsystem() != "platform") {
            ret.push_back(dev);
        }
    }

    return ret;
}
} // namespace SerialToolsInternal
