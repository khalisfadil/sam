#pragma once

#include <boost/asio.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <chrono>
#include <memory>
#include <thread>

#include <udpsocket.hpp>
#include <dataframe.hpp>
#include <lidarcallback.hpp>

class pipeline {
    public:
        static std::atomic<bool> Running_;                                                          // 
        static std::condition_variable Cv_;                                                          //
        std::atomic<int> DropLog_;                                                                   //
        boost::lockfree::spsc_queue<std::string, boost::lockfree::capacity<16>> LogQueue_;
        boost::lockfree::spsc_queue<LidarFrame, boost::lockfree::capacity<16>> BuffLidarFrame_;         //

        explicit pipeline(const std::string& odom_json_path, const std::string& lidar_json_path, const LidarCallback::LidarPreset& lidar_preset, uint16_t N = 1);
        static void signalHandler(int signal);
        void setThreadAffinity(const std::vector<int>& coreIDs);
        void logMessage(const std::string& level, const std::string& message);
        void runLidarListenerRng19(boost::asio::io_context& ioContext, UdpSocketConfig udp_config, const std::vector<int>& allowedCores);
        void runLidarListenerLegacy(boost::asio::io_context& ioContext, UdpSocketConfig udp_config, const std::vector<int>& allowedCores); 

    private:

        // %              ... lidar callback
        LidarCallback LidarCallback_;
        uint16_t FrameId_= 0;                                           // track frame id from lidar
};