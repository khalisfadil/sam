#include <pipeline.hpp>

int main() {
    std::string config_json = "../cpp/sam/config/core/odom_config.json";
    std::string lidar_json = "../cpp/sam/config/lidar/20250828_2204_OS-2-128_992120000670.json";
    uint32_t lidar_packet_size = 24896;
    std::string udp_profile_lidar;
    uint16_t udp_port_lidar;
    std::string udp_dest;
    // Configure Lidar
    uint16_t push_step_size = 4;
    uint16_t chnl_strd = 4;
    // Generate UTC timestamp for filename
    auto now = std::chrono::system_clock::now();
    auto utc_time = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::gmtime(&utc_time), "%Y%m%d_%H%M%S");
    std::string timestamp = ss.str();
    std::string log_filename = "../eval/log/log_report_" + timestamp + ".txt";
    std::string gt_filename = "../eval/gt/gt_report_" + timestamp + ".txt";

    try {
        std::ifstream json_file(lidar_json);
        if (!json_file.is_open()) {
                throw std::runtime_error("[Main] Error: Could not open JSON file: " + lidar_json);
            return EXIT_FAILURE;
        }
        nlohmann::json metadata_;
        json_file >> metadata_;
        json_file.close(); // Explicitly close the file

        if (!metadata_.contains("lidar_data_format") || !metadata_["lidar_data_format"].is_object()) {
                throw std::runtime_error("Missing or invalid 'lidar_data_format' object");
            return EXIT_FAILURE;
        }
        if (!metadata_.contains("config_params") || !metadata_["config_params"].is_object()) {
                throw std::runtime_error("Missing or invalid 'config_params' object");
            return EXIT_FAILURE;
        }
        if (!metadata_.contains("beam_intrinsics") || !metadata_["beam_intrinsics"].is_object()) {
                throw std::runtime_error("Missing or invalid 'beam_intrinsics' object");
            return EXIT_FAILURE;
        }
        if (!metadata_.contains("lidar_intrinsics") || !metadata_["lidar_intrinsics"].is_object() ||
            !metadata_["lidar_intrinsics"].contains("lidar_to_sensor_transform")) {
                throw std::runtime_error("Missing or invalid 'lidar_intrinsics.lidar_to_sensor_transform'");
            return EXIT_FAILURE;
        }

        udp_profile_lidar = metadata_["config_params"]["udp_profile_lidar"].get<std::string>();
        udp_port_lidar = metadata_["config_params"]["udp_port_lidar"].get<uint16_t>();
        udp_dest = metadata_["config_params"]["udp_dest"].get<std::string>();

    } catch (const std::exception& e) {
        std::cerr << "[Main] Error parsing JSON: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    pipeline pipeline(config_json, lidar_json, LidarCallback::LidarPreset::BERLIN20250901, chnl_strd);
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = pipeline::signalHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, nullptr);
    sigaction(SIGTERM, &sigIntHandler, nullptr);

#ifdef DEBUG
    std::cout << "[Main] Starting pipeline processes..." << std::endl;
#endif

    // Switch-case based on udp_profile_lidar
    enum class LidarProfile { RNG19_RFL8_SIG16_NIR16, LEGACY, UNKNOWN };
    LidarProfile profile;
    if (udp_profile_lidar == "RNG19_RFL8_SIG16_NIR16") {
        profile = LidarProfile::RNG19_RFL8_SIG16_NIR16;
        lidar_packet_size = 24832;
    } else if (udp_profile_lidar == "LEGACY") {
        profile = LidarProfile::LEGACY;
        lidar_packet_size = 24896;
    } else {
        profile = LidarProfile::UNKNOWN;
    }

    // Configure the UDP socket
    UdpSocketConfig config_lidar;
    config_lidar.host = "192.168.75.11"; // Listen on interfaces
    config_lidar.port = udp_port_lidar;     // UDP port for sonar data
    config_lidar.bufferSize = lidar_packet_size; // Maximum UDP packet size
    config_lidar.receiveTimeout = std::chrono::milliseconds(5000); // 1-second timeout
    config_lidar.enableBroadcast = false; // Enable broadcast if sonar data is broadcasted
    config_lidar.multicastGroup = boost::asio::ip::address::from_string("239.168.74.10"); // Multicast group
    config_lidar.ttl = 1; // Set TTL for multicast
    config_lidar.reuseAddress = true; // Allow port reuse

    //##############################################
    try {
        std::vector<std::thread> threads;
        boost::asio::io_context ioContextPoints;
        switch (profile) {
            case LidarProfile::RNG19_RFL8_SIG16_NIR16:
#ifdef DEBUG
                std::cout << "[Main] Detected RNG19_RFL8_SIG16_NIR16 lidar udp profile." << std::endl;
#endif
                threads.emplace_back([&]() { pipeline.runLidarListenerRng19(ioContextPoints, config_lidar, std::vector<int>{0}); });
                break;

            case LidarProfile::LEGACY:
#ifdef DEBUG
                std::cout << "[Main] Detected LEGACY lidar udp profile." << std::endl;
#endif
                // Example: Adjust buffer size or port for LEGACY mode if needed
                // bufferSize = 16384; // Example adjustment
                threads.emplace_back([&]() { pipeline.runLidarListenerLegacy(ioContextPoints, config_lidar, std::vector<int>{0}); });
                break;

            case LidarProfile::UNKNOWN:
            default:
                std::cerr << "[Main] Error: Unknown or unsupported udp_profile_lidar: " << udp_profile_lidar << std::endl;
                return EXIT_FAILURE;
        }


        while (pipeline::Running_.load(std::memory_order_acquire)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        ioContextPoints.stop();

        for (auto& thread : threads) {
            if (thread.joinable()) {
                thread.join();
            }
        }
        // --- SAFE SHUTDOWN POINT ---
        std::cout << "[Main] All threads joined. Saving final results..." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error: [Main] " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "[Main] All processes stopped. Exiting program." << std::endl;
    return EXIT_SUCCESS;
}
    
   
