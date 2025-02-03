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
#include <future>
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

class BelugaSerial {
  public:
    BelugaSerial();

    explicit BelugaSerial(
        const std::string &port, BaudRate baud = BAUD_115200,
        const std::chrono::milliseconds &timeout =
            std::chrono::milliseconds(2000),
        const std::chrono::milliseconds &serial_timeout =
            std::chrono::milliseconds(100),
        uint32_t max_lines_read = 16,
        void (*neighbor_updates_func)(const std::vector<BelugaNeighbor> &) =
            nullptr,
        void (*range_updates_func)(const std::vector<BelugaNeighbor> &) =
            nullptr,
        void (*range_event_func)(const BelugaFrame::RangeEvent &) = nullptr,
        int (*logger)(const char *, ...) = nullptr);
    ~BelugaSerial();

    std::string start_uwb();
    std::string stop_uwb();

    std::string start_ble();
    std::string stop_ble();

    std::string id(int id_);
    std::string id();

    std::string bootmode(int mode);
    std::string bootmode();

    std::string rate(int rate_);
    std::string rate();

    std::string channel(int channel_);
    std::string channel();

    std::string reset();

    std::string timeout(int timeout_);
    std::string timeout();

    std::string tx_power(int power);
    std::string tx_power(int stage, int coarse_gain, int fine_gain);
    std::string tx_power();

    std::string stream_mode(int updates_only);
    std::string stream_mode();

    std::string twr_mode(int mode);
    std::string twr_mode();

    std::string led_mode(int mode);
    std::string led_mode();

    std::string reboot();

    std::string pwr_amp(int mode);
    std::string pwr_amp();

    std::string antenna(int antenna);
    std::string antenna();

    std::string time();

    std::string format(int mode);
    std::string format();

    std::string deepsleep();

    std::string datarate(int rate_);
    std::string datarate();

    std::string preamble(int preamble);
    std::string preamble();

    std::string pulserate(int pr);
    std::string pulserate();

    void start();
    void stop();
    void close();

    bool get_neighbors(std::vector<BelugaNeighbor> &list);
    void get_ranges(std::vector<BelugaNeighbor> &list);
    BelugaFrame::RangeEvent get_range_event();

  private:
    int (*_logger)(const char *, ...) = nullptr;
    Serial::Serial _serial;
    uint32_t _read_max_lines = 16;
    std::chrono::milliseconds _timeout{};
    BelugaNeighborList _neighbors;

    BelugaQueue<std::vector<BelugaNeighbor>, 1, true> _neighbor_queue;
    void (*_neighbor_callback)(const std::vector<BelugaNeighbor> &) = nullptr;

    BelugaQueue<std::vector<BelugaNeighbor>, 1, true> _range_queue;
    void (*_range_callback)(const std::vector<BelugaNeighbor> &) = nullptr;

    BelugaQueue<BelugaFrame::RangeEvent, 1, true> _range_event_queue;
    void (*_range_event_callback)(const BelugaFrame::RangeEvent &) = nullptr;

    void _initialize(
        BaudRate baud = BAUD_115200,
        const std::chrono::milliseconds &timeout =
            std::chrono::milliseconds(2000),
        const std::chrono::milliseconds &serial_timeout =
            std::chrono::milliseconds(100),
        uint32_t max_lines_read = 16, const std::string &port = "",
        void (*neighbor_updates_func)(const std::vector<BelugaNeighbor> &) =
            nullptr,
        void (*range_updates_func)(const std::vector<BelugaNeighbor> &) =
            nullptr,
        void (*range_event_func)(const BelugaFrame::RangeEvent &) = nullptr,
        int (*logger)(const char *, ...) = nullptr);

    void _log(const char *msg);

    void _publish_neighbor_update();
    void _publish_range_update();
    void _publish_range_event(BelugaFrame::RangeEvent event);
    void _publish_response(std::string &response);

    void _process_reboot(const std::string &payload);

    static void
    _find_ports(const std::vector<target_pair> &valid,
                std::map<target_pair, std::vector<std::string>> &avail_ports);

    void _process_frames();

    void _process_rx_buffer(std::vector<uint8_t> &buf);
    void _read_serial();

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

    pid_t pid{};
};
} // namespace BelugaSerial

#endif // BELUGA_SERIAL_BELUGA_SERIAL_HPP
