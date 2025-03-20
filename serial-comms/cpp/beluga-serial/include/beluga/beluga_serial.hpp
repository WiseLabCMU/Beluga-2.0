/**
 * @file beluga_serial.hpp
 *
 * @brief
 *
 * @date 1/30/25
 *
 * @author tom
 */

#ifndef BELUGA_SERIAL_BELUGA_SERIAL_HPP
#define BELUGA_SERIAL_BELUGA_SERIAL_HPP

#include <atomic>
#include <beluga/beluga_event.hpp>
#include <beluga/beluga_frame.hpp>
#include <beluga/beluga_neighbor_list.hpp>
#include <beluga/beluga_queue.hpp>
#include <condition_variable>
#include <cstdarg>
#include <functional>
#include <future>
#include <mutex>
#include <serial/serial.hpp>
#include <serial/tools/list_ports.hpp>
extern "C" {
#include <unistd.h>
};

namespace BelugaSerial {
typedef std::pair<std::string, std::string> target_pair;

class FileNotFoundError : std::exception {
  public:
    explicit FileNotFoundError(const char *msg);

    [[nodiscard]] const char *what() const noexcept override;

  private:
    std::string _msg;
};

struct BelugaSerialAttributes {
    std::string port = ""; // NOLINT(*-redundant-string-init)
    BaudRate baud = BAUD_115200;
    std::chrono::milliseconds timeout = std::chrono::milliseconds(2000);
    std::chrono::milliseconds serial_timeout = std::chrono::milliseconds(100);
    std::function<void(const std::vector<BelugaNeighbor> &)>
        neighbor_update_cb = nullptr;
    std::function<void(const std::vector<BelugaNeighbor> &)> range_updates_cb =
        nullptr;
    std::function<void(const RangeEvent &)> range_event_cb = nullptr;
    std::function<int(const char *, va_list)> logger_cb = nullptr;
};

class BelugaSerial {
  public:
    BelugaSerial();

    explicit BelugaSerial(const BelugaSerialAttributes &attr);
    ~BelugaSerial();

    void register_resync_cb(std::function<void()> cb);
    void swap_port(const std::string &port);

    std::string start_uwb();
    std::string stop_uwb();

    std::string start_ble();
    std::string stop_ble();

    std::string id(const std::string &id_ = "");
    std::string bootmode(const std::string &mode = "");
    std::string rate(const std::string &rate_ = "");
    std::string channel(const std::string &channel_);
    std::string reset();
    std::string timeout(const std::string &timeout_ = "");
    std::string txpower(const std::string &power = "");
    std::string streammode(const std::string &updates_only = "");
    std::string twrmode(const std::string &mode = "");
    std::string ledmode(const std::string &mode = "");
    std::string reboot();
    std::string pwramp(const std::string &mode = "");
    std::string antenna(const std::string &antenna = "");
    std::string time();
    std::string deepsleep();
    std::string datarate(const std::string &rate_ = "");
    std::string preamble(const std::string &preamble = "");
    std::string pulserate(const std::string &pr = "");
    std::string phr(const std::string &phr_ = "");
    std::string pac(const std::string &pac = "");
    std::string sfd(const std::string &sfd = "");
    std::string panid(const std::string &pan_id = "");

    void start();
    void stop();
    void close();

    bool get_neighbors(std::vector<BelugaNeighbor> &list);
    void get_ranges(std::vector<BelugaNeighbor> &list);
    RangeEvent get_range_event();

  private:
    std::function<int(const char *, va_list)> _logger_cb = nullptr;
    Serial::Serial _serial;
    std::chrono::milliseconds _timeout{};
    BelugaNeighborList _neighbors;
    bool _usbRemainsOpen = false;
    uint16_t _id{};

    // Private because we don't want to get car-bombed by the user
    std::string format(const std::string &mode = "");

    BelugaQueue<std::vector<BelugaNeighbor>, 1, true> _neighbor_queue;
    std::function<void(const std::vector<BelugaNeighbor> &)> _neighbor_cb =
        nullptr;

    BelugaQueue<std::vector<BelugaNeighbor>, 1, true> _range_queue;
    std::function<void(const std::vector<BelugaNeighbor> &)> _range_cb =
        nullptr;

    BelugaQueue<RangeEvent, 1, true> _range_event_queue;
    std::function<void(const RangeEvent &)> _range_event_cb = nullptr;

    void
    _initialize(const BelugaSerialAttributes &attr = BelugaSerialAttributes{});

    void _log(const char *msg, ...);

    void _publish_neighbor_update();
    void _publish_range_update();
    void _publish_range_event(RangeEvent event);
    void _publish_response(std::string &response);

    void _process_reboot(const std::string &payload);

    static void
    _find_ports(const std::vector<target_pair> &valid,
                std::map<target_pair, std::vector<std::string>> &avail_ports);

    void __process_frames();
    void _process_frames();

    void _process_rx_buffer(std::vector<uint8_t> &buf);
    void __read_serial();
    void _read_serial();

    static uint16_t _extract_id(const std::string &s);

    enum ReconnectStates {
        RECONNECT_FIND,
        RECONNECT_SLEEP,
        RECONNECT_CONNECT,
        RECONNECT_UPDATE_SKIPS,
        RECONNECT_NEXT,
        RECONNECT_GET_ID,
        RECONNECT_CHECK_ID,
        RECONNECT_DONE,

        RECONNECT_INVALID,
    };

    static std::vector<std::string>
    _find_port_candidates(const std::vector<std::string> &skip_list);
    bool _open_port(std::string &port);
    std::string _get_id_from_device();
    void __reconnect();
    void _reconnect();
    void _reboot();

    std::string _send_command(const std::string &command);

    struct task_t {
        std::packaged_task<void()> task;
        std::thread thread;
    };

    std::atomic_bool _tasks_running = false;

    task_t _rx_task;
    BelugaQueue<BelugaFrame::DecodedFrame, 10, true> _batch_queue;

    task_t _processing_task;
    BelugaQueue<std::string> _response_queue;
    Event _command_sent;
    Event _reboot_done;

    std::function<void()> _time_resync = nullptr;

    std::recursive_mutex _serial_lock;
};
} // namespace BelugaSerial

#endif // BELUGA_SERIAL_BELUGA_SERIAL_HPP
