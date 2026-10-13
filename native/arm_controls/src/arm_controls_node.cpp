/*!
 * @file arm_controls_node.cpp
 * @brief Main entry point for the robot device control application.
 */

#include <algorithm>
#include <atomic>
#include <boost/program_options.hpp>
#include <cerrno>
#include <csignal>
#include <iostream>
#include <memory>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include "arm_controls_command_line_args.hpp"
#include "arm_controls.hpp"
#include "arm_controls_device.hpp"
#include "arm_controls_device_config.hpp"
#include "arm_controls_exception.hpp"
#include "arm_controls_profile.hpp"

void arm_controls_signal_handler(int signum) {
    g_terminate_signal_received = signum;
}

namespace {

class ParentLivenessMonitor {
   public:
    ParentLivenessMonitor() = default;
    ParentLivenessMonitor(const ParentLivenessMonitor&) = delete;
    ParentLivenessMonitor& operator=(const ParentLivenessMonitor&) = delete;

    ~ParentLivenessMonitor() { stop(); }

    void start(int fd) {
        if (fd < 0) {
            return;
        }
        fd_ = fd;
        thread_ = std::thread([this] { monitor(); });
    }

    void stop() {
        stop_requested_.store(true);
        if (thread_.joinable()) {
            thread_.join();
        }
        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }
    }

   private:
    void request_graceful_shutdown() const {
        if (!stop_requested_.load()) {
            // Deliver the same signal as Popen.terminate(). The existing
            // handler leaves device teardown on the main control thread.
            (void)kill(getpid(), SIGTERM);
        }
    }

    void monitor() {
        pollfd descriptor{fd_, POLLIN, 0};
        while (!stop_requested_.load()) {
            descriptor.revents = 0;
            const int result = poll(&descriptor, 1, 100);
            if (result == 0) {
                continue;
            }
            if (result < 0) {
                if (errno == EINTR) {
                    continue;
                }
                request_graceful_shutdown();
                return;
            }
            if ((descriptor.revents & (POLLHUP | POLLERR | POLLNVAL)) != 0) {
                request_graceful_shutdown();
                return;
            }
            if ((descriptor.revents & POLLIN) != 0) {
                char byte = 0;
                const ssize_t bytes_read = read(fd_, &byte, sizeof(byte));
                if (bytes_read == 0 || (bytes_read < 0 && errno != EINTR)) {
                    request_graceful_shutdown();
                    return;
                }
            }
        }
    }

    int fd_ = -1;
    std::atomic<bool> stop_requested_{false};
    std::thread thread_;
};

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, arm_controls_signal_handler);
    std::signal(SIGHUP, arm_controls_signal_handler);
    std::signal(SIGTERM, arm_controls_signal_handler);
    // Python owns the read end of stdout. If it exits abruptly, logging during
    // parent-death cleanup must see EPIPE instead of terminating the node before
    // p_device->stop() runs.
    std::signal(SIGPIPE, SIG_IGN);

    // Own the device instance for the entire scope of main (including catch
    // blocks).
    std::unique_ptr<Device> p_device;
    ParentLivenessMonitor parent_liveness_monitor;
    bool strict_exit_failure = false;

    try {
        CommandLineArgs cla(argc, argv);
        parent_liveness_monitor.start(cla.parent_liveness_fd);

        g_info_manager.set_info_level((InfoLevel)cla.info_level);
        g_info_manager.add_groups(cla.info_groups, ',');

        ReturnCode return_code;

        ARM_CONTROLS_INFO("main()", InfoLevel::ESSENTIAL_0,
                "Processing configuration files...");

        DeviceConfig device_config_model;
        return_code = device_config_model.init_config_model(cla);
        if (return_code != ReturnCode::SUCCESS) {
            ARM_CONTROLS_ERROR("Failed to read device model configuration");
            return -1;
        }

        DeviceConfig device_config_individual;
        return_code = device_config_individual.init_config_individual(cla);
        if (return_code != ReturnCode::SUCCESS) {
            ARM_CONTROLS_ERROR("Failed to read device individual configuration");
            return -1;
        }

        ARM_CONTROLS_INFO("main()", InfoLevel::ESSENTIAL_0, "Creating device...");

        p_device.reset(Device::new_device(device_config_model,
                                          device_config_individual, cla));
        if (!p_device) {
            ARM_CONTROLS_ERROR("Failed to create device");
            return -1;
        }

        p_device->set_topic_joystick_name(cla.topic_joystick);

        ARM_CONTROLS_INFO("main()", InfoLevel::ESSENTIAL_0, "Initializing device...");

        return_code = p_device->init(cla, argc, argv);
        if (return_code != ReturnCode::SUCCESS) {
            ARM_CONTROLS_ERROR("Device initialization failed");
            return -1;
        }

        ARM_CONTROLS_INFO("main()", InfoLevel::ESSENTIAL_0, "Starting devices...");

        return_code = p_device->start(cla.baud_rate);
        if (return_code != ReturnCode::SUCCESS) {
            // Communication failures should not keep the process running for
            // long.
            ARM_CONTROLS_ERROR("Device start() failed: error code=%d", return_code);
            return -1;
        }

        // Final servo error-state check after the whole start() sequence. The
        // in-start() verification can pass and a servo can still latch a
        // communication-loss error afterwards (e.g. a stale RAM protection
        // window expiring while a slow sibling servo was enabling). A latched
        // DM servo keeps reporting positions but produces no torque, so
        // without this check the joint would silently collapse under gravity
        // once commands stream. verify_servos_operational() probes for fresh
        // status frames, attempts one re-enable per latched DM servo, and
        // fails the startup if a servo stays in an error state. Must run
        // BEFORE arm_comm_loss_protection(): the recovery re-enable disarms
        // the DM TIMEOUT register, so arming has to stay the last step.
        return_code = p_device->verify_servos_operational();
        if (return_code != ReturnCode::SUCCESS) {
            ARM_CONTROLS_ERROR("Pre-loop servo verification failed: error code=%d", return_code);
            if (cla.no_recovery_motion) p_device->stop();
            return -1;
        }

        // Assert the servo communication-loss policy only now, when every
        // servo is enabled and verified and the command stream starts within
        // milliseconds. The window is per-device (wants_comm_loss_stop()):
        // armed for velocity/torque-commanded servos (stop a runaway), and
        // disarmed for position-commanded servos (keep holding the last
        // position instead of collapsing detorqued). Failures are loud but
        // non-fatal: the servos stay controllable, just with an unasserted
        // policy.
        return_code = p_device->arm_comm_loss_protection();
        if (return_code != ReturnCode::SUCCESS) {
            ARM_CONTROLS_ERROR("Failed to arm communication-loss protection: error code=%d", return_code);
            if (cla.no_recovery_motion) {
                p_device->stop();
                return -1;
            }
        }

        int capability_flags = cla.no_recovery_motion ? ARM_CONTROLS_CAP_NO_RECOVERY_MOTION : ARM_CONTROLS_CAP_MOVE_TO_READY;
        if (cla.role == Role::FOLLOWER) {
            capability_flags |= ARM_CONTROLS_CAP_DIRECT_COMMAND | ARM_CONTROLS_CAP_LIVE_INPUT;
        } else if (p_device->is_read_only()) {
            // Read-only leader (passive encoders, e.g. ARX_ENC): cannot produce torque,
            // so gravity compensation and force feedback are not available. The arm
            // still streams joint positions as a leader, and move-to-ready remains a
            // harmless no-op (the device marks itself ready immediately).
        } else {
            capability_flags |= ARM_CONTROLS_CAP_GRAVITY_COMP | ARM_CONTROLS_CAP_FORCE_FEEDBACK;
        }
        std::vector<int> handshake_data{
            ARM_CONTROLS_PROTOCOL_VERSION_MAJOR,
            ARM_CONTROLS_PROTOCOL_VERSION_MINOR,
            capability_flags,
        };

        ARM_CONTROLS_INFO("main()", InfoLevel::ESSENTIAL_0,
                "Starting main control loop...");

        // Strict callers must prove continued liveness even if the first PUB/SUB
        // heartbeat is lost (or no client ever sends one). Start the finite
        // deadline now; later heartbeats refresh it. Legacy opt-in arming stays
        // unchanged for clients that do not implement heartbeat messages.
        if (cla.no_recovery_motion) p_device->note_client_heartbeat();

        bool informed_ready_now = false;
        // ZMQ PUB/SUB can drop early messages until subscribers finish
        // connecting ("slow joiner"). To make readiness robust, re-publish
        // DEVICE_INFO_READY_NOW for a short period after first ready.
        int ready_announce_loops_remaining = 0;
        const int ready_announce_total_loops =
            std::max(1, (int)cla.control_frequency);  // ~1 second
        const int ready_announce_interval_loops =
            std::max(1, (int)cla.control_frequency / 10);  // ~0.1 sec
        int handshake_announce_counter = 0;
        int servo_param_announce_counter = 0;

        while (g_terminate_signal_received == 0 && p_device->is_running()) {
            return_code = p_device->step();
            if (return_code != ReturnCode::SUCCESS) {
                if (cla.no_recovery_motion) strict_exit_failure = true;
                if (return_code == ReturnCode::HARDWARE_FAULT) {
                    ARM_CONTROLS_ERROR("Protective stop: servo reported a hardware fault; "
                             "see preceding HARDWARE FAULT message");
                } else if (return_code <= ReturnCode::SAFE_MODE) {
                    if (return_code == ReturnCode::SAFE_MODE_POS_BEHIND) {
                        ARM_CONTROLS_ERROR(
                            "Protective stop: Position target behind limit "
                            "triggered safe mode");
                    } else if (return_code ==
                               ReturnCode::SAFE_MODE_POS_EXCEED) {
                        ARM_CONTROLS_ERROR(
                            "Protective stop: Position limit exceeded "
                            "triggered safe mode");
                    } else if (return_code == ReturnCode::SAFE_MODE_VEL) {
                        ARM_CONTROLS_ERROR(
                            "Protective stop: Velocity limit exceeded "
                            "triggered safe mode");
                    } else if (return_code == ReturnCode::SAFE_MODE_TOR) {
                        ARM_CONTROLS_ERROR(
                            "Protective stop: Torque limit exceeded triggered "
                            "safe mode");
                    } else if (return_code == ReturnCode::SAFE_MODE_SIG) {
                        ARM_CONTROLS_ERROR(
                            "Protective stop: Servo signal loss triggered safe "
                            "mode");
                    } else if (return_code ==
                               ReturnCode::SAFE_MODE_TEMPERATURE) {
                        ARM_CONTROLS_ERROR(
                            "Protective stop: Temperature limit exceeded "
                            "triggered safe mode");
                    } else {
                        ARM_CONTROLS_ERROR("Protective stop: Unknown error code: %d",
                                 return_code);
                    }
                } else {
                    ARM_CONTROLS_WARN("Device step() failed: error code=%d", return_code);
                }

                // Graceful recovery: instead of breaking immediately (which would torque-off all
                // joints at the current pose and let a heavy arm fall), trigger the emergency
                // recovery state machine. The device will switch to a follower-like position
                // mode and slowly drive reachable joints to the ready position at ERROR speed
                // before the topic self-stops via mark_emergency_recovery_completed(). This is
                // the only failure path for joint errors; there is no opt-out for the legacy
                // immediate-park behavior (it was unsafe at any distance from home).
                if (!p_device->is_in_emergency_recovery()) {
                    // Use the joint id recorded by the device's read_hardware_values() path
                    // (set via set_last_failed_joint_id) so the UI dialog can name the
                    // specific failed joint instead of showing "joint -1".
                    const int failed_joint_id = p_device->last_failed_joint_id();
                    p_device->enter_emergency_recovery(return_code, failed_joint_id);
                    if (cla.no_recovery_motion) break;
                    // Continue stepping; do NOT break.
                } else {
                    // We already entered recovery and this iteration still failed. That is expected
                    // (e.g. continued joint drop-outs); the slow ready move keeps trying best-effort.
                    ARM_CONTROLS_WARN("Error during emergency recovery (rc=%d); continuing best-effort",
                            static_cast<int>(return_code));
                }
            }

            if ((handshake_announce_counter++ % std::max(1, cla.control_frequency / 2)) == 0) {
                p_device->publish_device_info(DEVICE_INFO_PROTOCOL_HANDSHAKE, nullptr, &handshake_data);
            }

            // Servo parameters: one joint per publish on a faster cadence (10 Hz),
            // round-robin. Pub/sub gives no replay, so the report repeats forever for
            // late subscribers — and the status sockets run with a tiny HWM (2), so a
            // per-joint burst would be dropped down to its tail (observed: the client
            // received 1 of 6 joints when all were published in one tick).
            if ((servo_param_announce_counter++ % std::max(1, cla.control_frequency / 10)) == 0) {
                p_device->publish_next_servo_param();
            }

            const bool ready_now = p_device->is_ready();
            if (!ready_now) {
                // Allow re-announcement when the device re-enters ready state
                // (e.g. after COMMAND_MOVE_TO_READY_POS).
                informed_ready_now = false;
            }

            if (ready_now) {
                if (informed_ready_now == false) {
                    // Start re-announcement window.
                    ready_announce_loops_remaining = ready_announce_total_loops;
                    informed_ready_now = true;
                }

                // Publish at the first ready tick and then periodically for ~1
                // second.
                if (ready_announce_loops_remaining > 0) {
                    const bool is_first_ready_publish =
                        (ready_announce_loops_remaining ==
                         ready_announce_total_loops);
                    const bool is_periodic_publish =
                        ((ready_announce_loops_remaining %
                          ready_announce_interval_loops) == 0);
                    if (is_first_ready_publish || is_periodic_publish) {
                        std::vector<int> ready_data;
                        const int completed_request_id =
                            p_device->completed_move_to_ready_request_id();
                        if (completed_request_id > 0) {
                            ready_data.push_back(completed_request_id);
                        }
                        return_code = p_device->publish_device_info(
                            DEVICE_INFO_READY_NOW, nullptr,
                            ready_data.empty() ? nullptr : &ready_data);
                        if (return_code != ReturnCode::SUCCESS) {
                            ARM_CONTROLS_ERROR(
                                "Failed to publish device ready status: error "
                                "code=%d",
                                return_code);
                            break;
                        }
                        if (is_first_ready_publish) {
                            ARM_CONTROLS_INFO("main()", InfoLevel::ESSENTIAL_0,
                                    "Device is ready: %s_%s",
                                    p_device->get_model().c_str(),
                                    p_device->get_id().c_str());
                        }
                    }
                    ready_announce_loops_remaining -= 1;
                }
            }

            if (!p_device) {
                ARM_CONTROLS_ERROR("Device pointer is null in main control loop");
                break;
            }
            p_device->sleep();
        }

        // A client-heartbeat fault can stop the topic inside a successful step.
        // It must remain an unsuccessful exit even if its status message is lost.
        if (cla.no_recovery_motion && p_device->is_in_emergency_recovery()) strict_exit_failure = true;
        const std::sig_atomic_t terminate_signal = g_terminate_signal_received;
        if (terminate_signal != 0) {
            if (cla.no_recovery_motion) strict_exit_failure = true;
            ARM_CONTROLS_INFO("main()", InfoLevel::ESSENTIAL_0,
                    "Signal handler called with signal: %d",
                    static_cast<int>(terminate_signal));
        }

        ARM_CONTROLS_INFO("main()", InfoLevel::ESSENTIAL_0,
                "Main control loop ended, shutting down device...");

        if (p_device) {
            ARM_CONTROLS_INFO("main()", InfoLevel::ESSENTIAL_0, "Stopping device...");
            const auto stop_rc = p_device->stop();
            if (cla.no_recovery_motion && stop_rc != ReturnCode::SUCCESS) strict_exit_failure = true;
            p_device.reset();
        }
        ARM_CONTROLS_INFO("main()", InfoLevel::ESSENTIAL_0, "Device stopped");

    } catch (const PiException& e) {
        std::cerr << "Caught PiException: " << e.what() << std::endl;

        ///< @todo Disabled throwing exception in ARM_CONTROLS_ERROR() to avoid abrupt
        ///< program termination

        if (p_device) {
            p_device->park_safely();
            p_device.reset();
        }

        return 1;

    } catch (const std::exception& e) {
        std::cerr << "Caught standard exception: " << e.what() << std::endl;

        // Check if the exception is related to "Address already in use" or ZMQ
        // binding
        std::string error_msg = e.what();
        if (error_msg.find("Address already in use") != std::string::npos ||
            error_msg.find("EADDRINUSE") != std::string::npos ||
            error_msg.find("ZMQ bind failed") != std::string::npos ||
            error_msg.find("ZMQ connect failed") != std::string::npos) {
            ARM_CONTROLS_ERROR("ZMQ address binding/connection failed");
            ARM_CONTROLS_ERROR("Error details: %s", error_msg.c_str());
            ARM_CONTROLS_ERROR(
                "This usually means a ZMQ port is already bound by another "
                "process");
            ARM_CONTROLS_ERROR(
                "Try checking if another arm_controls_node instance is running");
            ARM_CONTROLS_ERROR(
                "Or check whether the ZMQ ports are still in use: ss -ltnp");
        } else {
            // For other exceptions, also try to extract address information if
            // available
            ARM_CONTROLS_ERROR("Standard exception occurred: %s", error_msg.c_str());
        }

        if (p_device) {
            p_device->park_safely();
            p_device.reset();
        }

        return 1;

    } catch (...) {
        std::cerr << "Caught unknown exception" << std::endl;
        if (p_device) {
            p_device->park_safely();
            p_device.reset();
        }

        return 1;
    }

    ARM_CONTROLS_INFO("main()", InfoLevel::ESSENTIAL_0,
            "arm_controls_node terminated successfully");

    return strict_exit_failure ? 1 : 0;
}
