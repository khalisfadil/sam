#include <pipeline.hpp>

using json = nlohmann::json;

std::atomic<bool> pipeline::Running_{true};
std::condition_variable pipeline::Cv_;

// %            ... initgravity
pipeline::pipeline(const std::string& odom_json_path, const std::string& lidar_json_path, const LidarCallback::LidarPreset& lidar_preset, uint16_t N) 
    :  LidarCallback_(lidar_json_path, lidar_preset, N) {// You can initialize other members here too

#ifdef DEBUG
    logMessage("LOGGING", "pipeline initialized.");
#endif
}
// %            ... initgravity
void pipeline::signalHandler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        Running_.store(false, std::memory_order_release);
        Cv_.notify_all();
    }
}
// %            ... initgravity
void pipeline::setThreadAffinity(const std::vector<int>& coreIDs) {
    if (coreIDs.empty()) {return;}
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    const unsigned int maxCores = std::thread::hardware_concurrency();
    uint32_t validCores = 0;
    for (int coreID : coreIDs) {
        if (coreID >= 0 && static_cast<unsigned>(coreID) < maxCores) {
            CPU_SET(coreID, &cpuset);
            validCores |= (1 << coreID);
        }
    }
    if (!validCores) {return;}
    if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) != 0) {
        Running_.store(false); // Optionally terminate
    }
}
// %            ... initgravity
void pipeline::logMessage(const std::string& level, const std::string& message) {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::ostringstream oss;
    oss << "[" << std::put_time(std::gmtime(&now_time_t), "%Y-%m-%dT%H:%M:%SZ") << "] "
        << "[" << level << "] " << message << "\n";
    if (!LogQueue_.push(oss.str())) {
        DropLog_.fetch_add(1, std::memory_order_relaxed);
    }
}
// %            ... initgravity
void pipeline::runLidarListenerRng19(boost::asio::io_context& ioContext, UdpSocketConfig udp_config, const std::vector<int>& allowedCores) {
    setThreadAffinity(allowedCores);
    Running_.store(true, std::memory_order_release);
    // Validate UdpSocketConfig
    if (udp_config.host.empty() || udp_config.port == 0 || udp_config.bufferSize == 0) {
#ifdef DEBUG
        std::ostringstream oss;
        oss << "Lidar Listener: Invalid host, port, or buffer size. Host: " << udp_config.host 
            << ", Port: " << udp_config.port << ", Buffer: " << udp_config.bufferSize;
        logMessage("ERROR", oss.str());
#endif
        return;
    }
    if (udp_config.receiveTimeout && udp_config.receiveTimeout->count() <= 0) {
#ifdef DEBUG
        std::ostringstream oss;
        oss << "Lidar Listener: Invalid receive timeout: " << udp_config.receiveTimeout->count() << " ms";
        logMessage("ERROR", oss.str());
#endif
        return;
    }
    if (udp_config.multicastGroup && !udp_config.multicastGroup->is_multicast()) {
#ifdef DEBUG
        std::ostringstream oss;
        oss << "Lidar Listener: Invalid multicast group: " << udp_config.multicastGroup->to_string();
        logMessage("ERROR", oss.str());
#endif
        return;
    }
    try {
        // Data callback: Decode and push to SPSC buffer
        auto dataCallback = [this](SpanType packet_data) {
            try {
                LidarFrame TempLidarFrame;
                LidarCallback_.DecodePacketRng19(
                    std::vector<uint8_t>(packet_data.begin(), packet_data.end()), TempLidarFrame);
                if (TempLidarFrame.numberpoints > 0 && TempLidarFrame.frame_id != FrameId_) {
                    FrameId_ = TempLidarFrame.frame_id;
                    if (!BuffLidarFrame_.push(std::move(TempLidarFrame))) {
#ifdef DEBUG
                        logMessage("WARNING", "Lidar Listener: SPSC buffer push failed for frame " + 
                                   std::to_string(FrameId_));
#endif
                    } else {
#ifdef DEBUG
                        std::ostringstream oss;
                        oss << "Lidar Listener: Processed frame " << FrameId_ << " with " 
                            << TempLidarFrame.numberpoints << " points";
                        logMessage("LOGGING", oss.str());
#endif
                    }
                }
            } catch (const std::exception& e) {
#ifdef DEBUG
                std::ostringstream oss;
                oss << "Lidar Listener: Decode error: " << e.what();
                logMessage("WARNING", oss.str());
#endif
            }
        };
        // Error callback: Log errors and handle shutdown
        auto errorCallback = [this](const boost::system::error_code& ec) {
#ifdef DEBUG
            std::ostringstream oss;
            oss << "Lidar Listener: UDP error: " << ec.message() << " (value: " << ec.value() << ")";
            logMessage("WARNING", oss.str());
#endif
            if (ec == boost::asio::error::operation_aborted || !Running_.load(std::memory_order_acquire)) {
                Running_.store(false, std::memory_order_release);
            }
        };
        // Create UDP socket
        auto socket = UdpSocket::create(ioContext, udp_config, dataCallback, errorCallback);
#ifdef DEBUG
        std::ostringstream oss;
        oss << "Lidar Listener: Started on " << udp_config.host << ":" << udp_config.port 
            << " with buffer size " << udp_config.bufferSize;
        logMessage("LOGGING", oss.str());
#endif
        // Run io_context (single-threaded, non-blocking)
        while (Running_.load(std::memory_order_acquire)) {
            try {
                ioContext.run_one(); // Process one event at a time
            } catch (const std::exception& e) {
#ifdef DEBUG
                std::ostringstream oss;
                oss << "Lidar Listener: io_context exception: " << e.what();
                logMessage("WARNING", oss.str());
#endif
                if (Running_.load(std::memory_order_acquire)) {
                    ioContext.restart();
#ifdef DEBUG
                    logMessage("LOGGING", "Lidar Listener: io_context restarted.");
#endif
                } else {
                    break;
                }
            }
        }
        // Cleanup
        socket->stop();
        if (!ioContext.stopped()) {
            ioContext.stop();
#ifdef DEBUG
            logMessage("LOGGING", "Lidar Listener: Stopped");
#endif
        }

    } catch (const std::exception& e) {
#ifdef DEBUG
        std::ostringstream oss;
        oss << "Lidar Listener: Setup exception: " << e.what();
        logMessage("WARNING", oss.str());
#endif
        Running_.store(false, std::memory_order_release);
        if (!ioContext.stopped()) {
            ioContext.stop();
#ifdef DEBUG
            logMessage("LOGGING", "Lidar Listener: Stopped");
#endif
        }
    }
}
// %            ... initgravity
void pipeline::runLidarListenerLegacy(boost::asio::io_context& ioContext, UdpSocketConfig udp_config, const std::vector<int>& allowedCores) {
    setThreadAffinity(allowedCores);
    Running_.store(true, std::memory_order_release);
    // Validate UdpSocketConfig
    if (udp_config.host.empty() || udp_config.port == 0 || udp_config.bufferSize == 0) {
#ifdef DEBUG
        std::ostringstream oss;
        oss << "Lidar Listener: Invalid host, port, or buffer size. Host: " << udp_config.host 
            << ", Port: " << udp_config.port << ", Buffer: " << udp_config.bufferSize;
        logMessage("ERROR", oss.str());
#endif
        return;
    }
    if (udp_config.receiveTimeout && udp_config.receiveTimeout->count() <= 0) {
#ifdef DEBUG
        std::ostringstream oss;
        oss << "Lidar Listener: Invalid receive timeout: " << udp_config.receiveTimeout->count() << " ms";
        logMessage("ERROR", oss.str());
#endif
        return;
    }
    if (udp_config.multicastGroup && !udp_config.multicastGroup->is_multicast()) {
#ifdef DEBUG
        std::ostringstream oss;
        oss << "Lidar Listener: Invalid multicast group: " << udp_config.multicastGroup->to_string();
        logMessage("ERROR", oss.str());
#endif
        return;
    }
    try {
        // Data callback: Decode and push to SPSC buffer
        auto dataCallback = [this](SpanType packet_data) {
            try {
                LidarFrame TempLidarFrame;
                LidarCallback_.DecodePacketLegacy(
                    std::vector<uint8_t>(packet_data.begin(), packet_data.end()), TempLidarFrame);
                if (TempLidarFrame.numberpoints > 0 && TempLidarFrame.frame_id != FrameId_) {
                    FrameId_ = TempLidarFrame.frame_id;
                    if (!BuffLidarFrame_.push(std::move(TempLidarFrame))) {
#ifdef DEBUG
                        logMessage("WARNING", "Lidar Listener: SPSC buffer push failed for frame " + 
                                   std::to_string(FrameId_));
#endif
                    } else {
#ifdef DEBUG
                        std::ostringstream oss;
                        oss << "Lidar Listener: Processed frame " << FrameId_ << " with " 
                            << TempLidarFrame.numberpoints << " points";
                        logMessage("LOGGING", oss.str());
#endif
                    }
                }
            } catch (const std::exception& e) {
#ifdef DEBUG
                std::ostringstream oss;
                oss << "Lidar Listener: Decode error: " << e.what();
                logMessage("WARNING", oss.str());
#endif
            }
        };
        // Error callback: Log errors and handle shutdown
        auto errorCallback = [this](const boost::system::error_code& ec) {
#ifdef DEBUG
            std::ostringstream oss;
            oss << "Lidar Listener: UDP error: " << ec.message() << " (value: " << ec.value() << ")";
            logMessage("WARNING", oss.str());
#endif
            if (ec == boost::asio::error::operation_aborted || !Running_.load(std::memory_order_acquire)) {
                Running_.store(false, std::memory_order_release);
            }
        };
        // Create UDP socket
        auto socket = UdpSocket::create(ioContext, udp_config, dataCallback, errorCallback);
#ifdef DEBUG
        std::ostringstream oss;
        oss << "Lidar Listener: Started on " << udp_config.host << ":" << udp_config.port 
            << " with buffer size " << udp_config.bufferSize;
        logMessage("LOGGING", oss.str());
#endif
        // Run io_context (single-threaded, non-blocking)
        while (Running_.load(std::memory_order_acquire)) {
            try {
                ioContext.run_one(); // Process one event at a time
            } catch (const std::exception& e) {
#ifdef DEBUG
                std::ostringstream oss;
                oss << "Lidar Listener: io_context exception: " << e.what();
                logMessage("WARNING", oss.str());
#endif
                if (Running_.load(std::memory_order_acquire)) {
                    ioContext.restart();
#ifdef DEBUG
                    logMessage("LOGGING", "Lidar Listener: io_context restarted.");
#endif
                } else {
                    break;
                }
            }
        }
        // Cleanup
        socket->stop();
        if (!ioContext.stopped()) {
            ioContext.stop();
#ifdef DEBUG
            logMessage("LOGGING", "Lidar Listener: Stopped");
#endif
        }

    } catch (const std::exception& e) {
#ifdef DEBUG
        std::ostringstream oss;
        oss << "Lidar Listener: Setup exception: " << e.what();
        logMessage("WARNING", oss.str());
#endif
        Running_.store(false, std::memory_order_release);
        if (!ioContext.stopped()) {
            ioContext.stop();
#ifdef DEBUG
            logMessage("LOGGING", "Lidar Listener: Stopped");
#endif
        }
    }
}
// %            ... initgravity