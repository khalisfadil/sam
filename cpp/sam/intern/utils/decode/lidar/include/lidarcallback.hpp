// OusterLidarCallback.hpp
#pragma once

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include <cstdint>
#include <mutex>

#include <dataframe.hpp>

class LidarCallback {
    public:
        enum class LidarPreset {
            BERLIN20250730,
            GEHLSDORF20250410,
            BERLIN20250901
        };

        explicit LidarCallback(const std::string& json_path, const LidarPreset& T, uint16_t N = 1);
        explicit LidarCallback(const nlohmann::json& json_data, const LidarPreset& T, uint16_t N = 1);
        void DecodePacketRng19(const std::vector<uint8_t>& packet, LidarFrame& frame);
        void DecodePacketLegacy(const std::vector<uint8_t>& packet, LidarFrame& frame); 
        const LidarFrame& GetLatestFrame() const { return buffer_toggle_ ? data_buffer1_ : data_buffer2_; }
        const nlohmann::json& GetMetadata() const { return metadata_; }

    private:
        LidarPreset transform_preset_ = LidarPreset::BERLIN20250730;
        uint16_t channel_stride_ = 1; // Number of rows to skip (N), default to 1 (process all rows)
        uint16_t subset_channels_;    // Number of channels in subset tables (ceiling(pixels_per_column_ / N))

        nlohmann::json metadata_;
        // Original lookup tables
        std::vector<std::vector<double, Eigen::aligned_allocator<double>>> x_1_;
        std::vector<std::vector<double, Eigen::aligned_allocator<double>>> y_1_;
        std::vector<std::vector<double, Eigen::aligned_allocator<double>>> z_1_;
        std::vector<double> x_2_;
        std::vector<double> y_2_;
        std::vector<double> z_2_;
        std::vector<double> r_min_;
        std::vector<double> sin_beam_azimuths_;
        std::vector<double> cos_beam_azimuths_;
        std::vector<double> sin_beam_altitudes_;
        std::vector<double> cos_beam_altitudes_;
        std::vector<int> pixel_shifts_;
        // Subset lookup tables for channels that are multiples of N
        std::vector<std::vector<double, Eigen::aligned_allocator<double>>> x_1_subset_;
        std::vector<std::vector<double, Eigen::aligned_allocator<double>>> y_1_subset_;
        std::vector<std::vector<double, Eigen::aligned_allocator<double>>> z_1_subset_;
        std::vector<double> r_min_subset_;
        std::vector<double> sin_beam_azimuths_subset_;
        std::vector<double> cos_beam_azimuths_subset_;
        std::vector<double> sin_beam_altitudes_subset_;
        std::vector<double> cos_beam_altitudes_subset_;
        std::vector<int> pixel_shifts_subset_;
        std::vector<uint16_t> subset_c_ids_; // Maps subset indices to original c_id

        Eigen::Matrix4d lidar_to_sensor_transform_;
        double lidar_origin_to_beam_origin_mm_;
        size_t block_size_;
        size_t expected_size_;
        size_t PACKET_HEADER_BYTES = 32;
        size_t PACKET_FOOTER_BYTES = 32;
        size_t COLUMN_HEADER_BYTES = 12;
        size_t CHANNEL_STRIDE_BYTES = 12;
        size_t MEASUREMENT_BLOCK_STATUS_BYTES = 0;
        std::string udp_profile_lidar_ = "UNKNOWN";
        int columns_per_frame_ = 2048;
        int pixels_per_column_ = 128;
        int columns_per_packet_ = 16;
        uint16_t frame_id_ = 0;
        uint32_t number_points_ = 0;
        double latest_timestamp_s = 0.0;
        LidarFrame data_buffer1_;
        LidarFrame data_buffer2_;
        bool buffer_toggle_ = true;

        void Initialize();
        void ParseMetadata(const nlohmann::json& json_data);
        void SwapBuffer() { buffer_toggle_ = !buffer_toggle_; }
};

