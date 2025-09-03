/**
 * @file beluga_serial.hpp
 *
 * @brief Abstraction for serial communication with Beluga devices. This ensures
 * timely reading and processing of data from the serial port since this
 * abstraction does those tasks in a concurrent manner.
 *
 * @date 1/30/25
 *
 * @author Tom Schmitz \<tschmitz@andrew.cmu.edu\>
 */

#ifndef BELUGA_SERIAL_BELUGA_SERIAL_BASE_HPP
#define BELUGA_SERIAL_BELUGA_SERIAL_BASE_HPP

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

/// Exception for not being bale to find a file
class FileNotFoundError : std::exception {
  public:
    /**
     * @brief Constructor
     * @param msg The message to display
     */
    explicit FileNotFoundError(const char *msg);

    /**
     * @brief Returns the error message
     * @return The error message
     */
    [[nodiscard]] const char *what() const noexcept override;

  private:
    std::string _msg;
};

/**
 * Configuration attributes for BelugaSerial
 */
struct BelugaSerialAttributes {
    /**
     * The port to use
     * Default is empty
     */
    std::string port = ""; // NOLINT(*-redundant-string-init)

    /**
     * Baudrate to use
     * Default is 115200
     */
    BaudRate baud = BAUD_115200;

    /**
     * Timeout for sending commands
     * Default is 2000ms
     */
    std::chrono::milliseconds timeout = std::chrono::milliseconds(2000);

    /**
     * Timeout for serial reads
     * Default is 100ms
     */
    std::chrono::milliseconds serial_timeout = std::chrono::milliseconds(100);

    /**
     * Callback for neighbor updates
     * Default is nullptr
     * @note If this is left unset, the library will use a queue to store
     * updates.
     */
    std::function<void(const std::vector<BelugaNeighbor> &)>
        neighbor_update_cb = nullptr;

    /**
     * Callback for range updates
     * Default is nullptr
     * @note If this is left unset, the library will use a queue to store
     * updates.
     */
    std::function<void(const std::vector<BelugaNeighbor> &)> range_updates_cb =
        nullptr;

    /**
     * Callback for range events
     * Default is nullptr
     * @note If this is left unset, the library will use a queue to store
     * updates.
     */
    std::function<void(const RangeEvent &)> range_event_cb = nullptr;

    /**
     * Logger callback
     * Default is nullptr
     * @note If this is left unset, the library will not log anything.
     */
    std::function<int(const char *, va_list)> logger_cb = nullptr;

    /**
     * Callback for reporting UWB exchange drops
     * Default is nullptr
     * @note If this is left unset, then the library will not report UWB
     * exchange drops.
     */
    std::function<void(const BelugaFrame::DroppedUwbExchange &)>
        report_uwb_drops_cb = nullptr;

    /**
     * Callback for reporting unexpected reboots.
     * Default is nullptr
     * @note If this is left unset, then the library will not report
     * unexpected reboots.
     */
    std::function<void()> unexpected_reboot_event = nullptr;
};

/// Base manager for serial communication with Beluga nodes
class BelugaSerialBase {
  public:
    /**
     * Manufacturer and Product pair for Beluga devices
     */
    typedef std::pair<std::string, std::string> target_pair;

    /**
     * Default constructor
     */
    BelugaSerialBase();

    /**
     * Constructor with attributes
     * @param[in] attr The attributes to use
     */
    explicit BelugaSerialBase(const BelugaSerialAttributes &attr);

    /**
     * Destructor
     */
    ~BelugaSerialBase();

    /**
     * Register a callback for resynchronizing time
     * @param[in] cb The callback function to call to resync time. To unregister
     * a function, pass in nullptr.
     */
    void register_resync_cb(std::function<void()> cb);

    /**
     * Closes the previously opened port and opens the new port.
     * @param[in] port The new port to use
     * @throws std::invalid_argument if the port is not valid
     * @throws std::runtime_error if the receive and processing tasks are still
     * running
     */
    void swap_port(const std::string &port);

    /**
     * Sends a command for starting UWB
     * @return The command response
     */
    std::string start_uwb();

    /**
     * Sends a command for stopping UWB
     * @return The command response
     */
    std::string stop_uwb();

    /**
     * Sends a command for starting BLE
     * @return The command response
     */
    std::string start_ble();

    /**
     * Sends a command for stopping BLE
     * @return The command response
     */
    std::string stop_ble();

    /**
     * Sends a command for setting/getting the ID
     * @param[in] id The ID to set. If empty, it will get the current ID
     * @return The command response
     */
    std::string id(const std::string &id = "");

    /**
     * Sends a command for setting/getting the boot mode
     * @param[in] mode The boot mode to set. If empty, it will get the current
     * mode
     * @return The command response
     */
    std::string bootmode(const std::string &mode = "");

    /**
     * Sends a command for setting/getting the rate
     * @param[in] rate The rate to set. If empty, it will get the current rate
     * @return The command response
     */
    std::string rate(const std::string &rate = "");

    /**
     * Sends a command for setting/getting the channel
     * @param[in] channel The channel to set. If empty, it will get the current
     * channel
     * @return The command response
     */
    std::string channel(const std::string &channel);

    /**
     * Sends a command for resetting the device settings
     * @return The command response
     */
    std::string reset();

    /**
     * Sends a command for setting/getting the timeout
     * @param[in] timeout The timeout to set. If empty, it will get the current
     * timeout
     * @return The command response
     */
    std::string timeout(const std::string &timeout = "");

    /**
     * Sends a command for setting/getting the transmit power
     * @param[in] power The power to set. If empty, it will get the current
     * power
     * @return The command response
     */
    std::string txpower(const std::string &power = "");

    /**
     * Sends a command for setting/getting the stream mode
     * @param[in] updates_only The mode to set. If empty, it will get the
     * current mode
     * @return The command response
     */
    std::string streammode(const std::string &updates_only = "");

    /**
     * Sends a command for setting/getting the TWR mode
     * @param[in] mode The mode to set. If empty, it will get the current mode
     * @return The command response
     */
    std::string twrmode(const std::string &mode = "");

    /**
     * Sends a command for setting/getting the LED mode
     * @param[in] mode The mode to set. If empty, it will get the current mode
     * @return The command response
     */
    std::string ledmode(const std::string &mode = "");

    /**
     * Sends a command for rebooting the device
     * @return The command response
     */
    std::string reboot();

    /**
     * Sends a command for setting/getting the power amplifier mode
     * @param[in] mode The mode to set. If empty, it will get the current mode
     * @return The command response
     */
    std::string pwramp(const std::string &mode = "");

    /**
     * Sends a command for setting/getting the BLE antenna
     * @param[in] antenna The antenna to set. If empty, it will get the current
     * antenna in use
     * @return The command response
     */
    std::string antenna(const std::string &antenna = "");

    /**
     * Sends a command for getting the time
     * @return The command response
     */
    std::string time();

    /**
     * Sends a command for putting the device into deep sleep
     * @return The command response
     */
    std::string deepsleep();

    /**
     * Sends a command for setting/getting the data rate
     * @param[in] rate The datarate to set. If empty, it will get the current
     * data rate
     * @return The command response
     */
    std::string datarate(const std::string &rate = "");

    /**
     * Sends a command for setting/getting the preamble length
     * @param[in] preamble The preamble to set. If empty, it will get the
     * current preamble length
     * @return The command response
     */
    std::string preamble(const std::string &preamble = "");

    /**
     * Sends a command for setting/getting the pulse rate
     * @param[in] pr The pulse rate to set. If empty, it will get the current
     * pulse rate
     * @return The command response
     */
    std::string pulserate(const std::string &pr = "");

    /**
     * Sends a command for setting/getting the PHR
     * @param[in] phr The PHR to set. If empty, it will get the current PHR
     * @return The command response
     */
    std::string phr(const std::string &phr = "");

    /**
     * Sends a command for setting/getting the PAC size
     * @param[in] pac The PAC to set. If empty, it will get the current PAC size
     * @return The command response
     */
    std::string pac(const std::string &pac = "");

    /**
     * Sends a command for setting/getting the SFD
     * @param[in] sfd The SFD to set. If empty, it will get the current SFD
     * @return The command response
     */
    std::string sfd(const std::string &sfd = "");

    /**
     * Sends a command for setting/getting the PAN ID
     * @param[in] pan_id The PAN ID to set. If empty, it will get the current
     * PAN ID
     * @return The command response
     */
    std::string panid(const std::string &pan_id = "");

    /**
     * Sends a command for setting/getting the neighbor eviction scheme
     * @param[in] scheme The eviction scheme to set. If empty, it will get the
     * current eviction scheme
     * @return The command response
     */
    std::string evict(const std::string &scheme = "");

    /**
     * Sends a command for setting/getting the verbose mode
     * @param[in] mode The mode to set. If empty, it will get the current
     * verbose mode setting.
     * @return The command response
     */
    std::string verbose(const std::string &mode = "");

    /**
     * Retrieves the current status of the Beluga node.
     * @return The command response
     */
    std::string status();

    /**
     * Retrieves the current version of the Beluga node.
     * @return The command response
     */
    std::string version();

    /**
     * Starts the serial reception and processing tasks
     * @throws std::runtime_error if the receive and processing tasks are
     * already running
     */
    void start();

    /**
     * Stops the serial reception and processing tasks
     * @throws std::runtime_error if the tasks are unable to terminate
     */
    void stop();

    /**
     * Stops the serial reception and processing tasks and close the port
     * @throws std::runtime_error if the tasks are unable to terminate
     */
    void close();

    /**
     * Retrieves the list of neighbors from the queue.
     * @param[in,out] list List of neighbors to populate
     * @return `true` if a neighbor update was available, `false` otherwise
     * @throws BelugaQueueException if there was an error retrieving the
     * neighbors and the error was not because the queue was empty
     * @note This will always return `false` if the neighbor updates callback is
     * set.
     */
    bool get_neighbors(std::vector<BelugaNeighbor> &list);

    /**
     * Retrieves the list of ranges from the queue.
     * @param[in,out] list List of ranges to populate
     * @throws BelugaQueueException if there was an error retrieving the ranges
     * and the error was not because the queue was empty
     * @note The list will always be empty if the range updates callback is set.
     */
    void get_ranges(std::vector<BelugaNeighbor> &list);

    /**
     * Retrieves a ranging event from the queue.
     * @return The ranging event
     * @throws BelugaQueueException if there was an error retrieving the event
     * and the error was not because the queue was empty
     * @note The event will always be empty if the range event callback is set.
     */
    RangeEvent get_range_event();

  protected:
    /**
     * Virtual method for publishing neighbor updates.
     *
     * @note This method must be overridden.
     */
    virtual void publish_neighbor_update_() = 0;

    /**
     * Virtual method for publishing range updates.
     *
     * @note This method must be overridden.
     */
    virtual void publish_range_update_() = 0;

    /**
     * Virtual method for updating the neighbor list.
     * @param[in] updates The updates to the neighbor list.
     *
     * @note This method must be overridden.
     */
    virtual void update_neighbor_list_(
        const std::vector<BelugaFrame::NeighborUpdate> &updates) = 0;

    /**
     * Virtual method for removing a node from the neighbor list.
     * @param[in] id The ID of the node being removed.
     *
     * @note This method must be overridden.
     */
    virtual void remove_from_neighbor_list_(uint32_t id) = 0;

    /**
     * Virtual method for clearing the neighbor list.
     *
     * @note This method must be overridden.
     */
    virtual void clear_neighbor_list_() = 0;

    /**
     * Method for publishing neighbor updates to the appropriate location. This
     * must be called within @ref publish_neighbor_update_().
     * @param[in] updates The updates to publish.
     */
    void _publish_neighbor_updates_(std::vector<BelugaNeighbor> &updates);

    /**
     * Method for publishing range updates to the appropriate location. This
     * must be called within @ref publish_range_update_().
     * @param[in] updates The updates to publish.
     */
    void _publish_range_updates_(std::vector<BelugaNeighbor> &updates);

  private:
    std::function<int(const char *, va_list)> _logger_cb = nullptr;
    Serial::Serial _serial;
    std::chrono::milliseconds _timeout{};
    bool _usb_remains_open = false;
    uint16_t _id{};

    // Private because we don't want to get car-bombed by the user
    std::string _format(const std::string &mode = "");

    BelugaQueue<RangeEvent, 1, true> _range_event_queue;
    std::function<void(const RangeEvent &)> _range_event_cb = nullptr;

    std::function<void()> _unexpected_reboot_event_cb = nullptr;

    void
    _initialize(const BelugaSerialAttributes &attr = BelugaSerialAttributes{});

    void _log(const char *msg, ...);

    void _publish_range_event(RangeEvent event);
    void _publish_response(std::string &response);
    void _publish_uwb_exchange_drop(BelugaFrame::DroppedUwbExchange &drop_info);

    void _process_reboot(const std::string &payload);

    static void
    _find_ports(const std::vector<target_pair> &valid,
                std::map<target_pair, std::vector<std::string>> &avail_ports);

    void _process_frames_helper();
    void _process_frames();

    void _process_rx_buffer(std::vector<uint8_t> &buf);
    void _read_serial_helper();
    void _read_serial();

    BelugaQueue<std::vector<BelugaNeighbor>, 1, true> _neighbor_queue;
    std::function<void(const std::vector<BelugaNeighbor> &)> _neighbor_cb =
        nullptr;

    BelugaQueue<std::vector<BelugaNeighbor>, 1, true> _range_queue;
    std::function<void(const std::vector<BelugaNeighbor> &)> _range_cb =
        nullptr;

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
    void _reconnect_helper();
    void _reconnect();
    void _reboot();
    void _notify_unexpected_reboot();
    void _resync_time();

    std::string _send_command(const std::string &command);

    struct Task {
        std::packaged_task<void()> task;
        std::thread thread;
    };

    std::atomic_bool _tasks_running = false;

    Task _rx_task;
    BelugaQueue<BelugaFrame::DecodedFrame, 10, true> _batch_queue;

    Task _processing_task;
    BelugaQueue<std::string> _response_queue;
    Event _command_sent;
    Event _reboot_done;

    std::function<void()> _time_resync = nullptr;

    std::recursive_mutex _serial_lock;

    std::function<void(const BelugaFrame::DroppedUwbExchange &)>
        _report_uwb_drops = nullptr;
};
} // namespace BelugaSerial

#endif // BELUGA_SERIAL_BELUGA_SERIAL_BASE_HPP
