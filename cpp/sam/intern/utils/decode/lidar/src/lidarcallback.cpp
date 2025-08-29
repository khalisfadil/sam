#include <fstream>
#include <stdexcept>
#include <cmath>
#include <cstring> 
#include <iostream> 
#include <algorithm> 

#include <lidarcallback.hpp>

#ifdef __AVX2__
#include <immintrin.h>
#endif
#include <endian.h> 

using json = nlohmann::json;

// %            ... lidarcallback constructor
LidarCallback::LidarCallback(const std::string& json_path, const LidarPreset& T, uint16_t N) 
    : transform_preset_(T), channel_stride_(N) {
    if (N == 0) {
        throw std::runtime_error("Channel stride N must be positive");
    }
    if (N != 1 && N != 2 && N != 4 && N != 8 && N != 16) {
        throw std::runtime_error("Channel stride N must be one of 1, 2, 4, 8, or 16, got " + std::to_string(N));
    }
    std::ifstream file(json_path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open JSON file: " + json_path);
    }
    json json_data;
    try {
        file >> json_data;
    } catch (const json::parse_error& e) {
        throw std::runtime_error("JSON parse error in " + json_path + ": " + e.what());
    }
    ParseMetadata(json_data);
    if (N > static_cast<uint16_t>(pixels_per_column_)) {
        throw std::runtime_error("Channel stride N (" + std::to_string(N) + ") exceeds pixels_per_column_ (" + std::to_string(pixels_per_column_) + ")");
    }
    Initialize();
}
// %            ... sensor fusion constructor
LidarCallback::LidarCallback(const json& json_data, const LidarPreset& T, uint16_t N) 
    : transform_preset_(T), channel_stride_(N) {
    if (N == 0) {
        throw std::runtime_error("Channel stride N must be positive");
    }
    if (N != 1 && N != 2 && N != 4 && N != 8 && N != 16) {
        throw std::runtime_error("Channel stride N must be one of 1, 2, 4, 8, or 16, got " + std::to_string(N));
    }
    ParseMetadata(json_data);
    if (N > static_cast<uint16_t>(pixels_per_column_)) {
        throw std::runtime_error("Channel stride N (" + std::to_string(N) + ") exceeds pixels_per_column_ (" + std::to_string(pixels_per_column_) + ")");
    }
    Initialize();
}
// %            ... sensor fusion constructor
void LidarCallback::ParseMetadata(const json& json_data_param) {
    if (!json_data_param.is_object()) {
        throw std::runtime_error("JSON data must be an object");
    }
    // Store the provided JSON data; subsequent accesses use the metadata_ member.
    metadata_ = json_data_param;

    try {
        if (!metadata_.contains("lidar_data_format") || !metadata_["lidar_data_format"].is_object()) {
            throw std::runtime_error("Missing or invalid 'lidar_data_format' object");
        }
        if (!metadata_.contains("config_params") || !metadata_["config_params"].is_object()) {
            throw std::runtime_error("Missing or invalid 'config_params' object");
        }
        if (!metadata_.contains("beam_intrinsics") || !metadata_["beam_intrinsics"].is_object()) {
            throw std::runtime_error("Missing or invalid 'beam_intrinsics' object");
        }
        if (!metadata_.contains("lidar_intrinsics") || !metadata_["lidar_intrinsics"].is_object() ||
            !metadata_["lidar_intrinsics"].contains("lidar_to_sensor_transform")) {
            throw std::runtime_error("Missing or invalid 'lidar_intrinsics.lidar_to_sensor_transform'");
        }

        columns_per_frame_ = metadata_["lidar_data_format"]["columns_per_frame"].get<int>();
        pixels_per_column_ = metadata_["lidar_data_format"]["pixels_per_column"].get<int>();
        columns_per_packet_ = metadata_["config_params"]["columns_per_packet"].get<int>();
        udp_profile_lidar_ = metadata_["config_params"]["udp_profile_lidar"].get<std::string>();

        const auto& beam_intrinsics = metadata_["beam_intrinsics"];
        lidar_origin_to_beam_origin_mm_ = beam_intrinsics["lidar_origin_to_beam_origin_mm"].get<double>();

        const auto& pixel_shift_by_row = metadata_["lidar_data_format"]["pixel_shift_by_row"];
        if (!pixel_shift_by_row.is_array() || pixel_shift_by_row.size() != static_cast<size_t>(pixels_per_column_)) {
            throw std::runtime_error("'pixel_shift_by_row' must be an array of " + std::to_string(pixels_per_column_) + " elements");
        }
        pixel_shifts_.resize(pixels_per_column_);
        for (int i = 0; i < pixels_per_column_; ++i) {
            pixel_shifts_[i] = pixel_shift_by_row[i].get<int>();
        }

        const auto& lidar_transform_json = metadata_["lidar_intrinsics"]["lidar_to_sensor_transform"];
        if (!lidar_transform_json.is_array() || lidar_transform_json.size() != 16) {
            throw std::runtime_error("'lidar_to_sensor_transform' must be an array of 16 elements");
        }
        
        // Eigen::Matrix4d raw_transform = Eigen::Matrix4d::Identity(); // Not strictly needed if lidar_to_sensor_transform_ is filled directly
        lidar_to_sensor_transform_ = Eigen::Matrix4d::Identity();
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                double value = lidar_transform_json[i * 4 + j].get<double>();
                // raw_transform(i, j) = value; // if needed for other purposes
                // Scale translation components (last column, rows 0-2) from mm to m
                if (j == 3 && i < 3) {
                    lidar_to_sensor_transform_(i, j) = value * 0.001;
                } else {
                    lidar_to_sensor_transform_(i, j) = value;
                }
            }
        }
    } catch (const json::exception& e) {
        throw std::runtime_error("JSON parsing error in metadata: " + std::string(e.what()));
    }
}
// %            ... initialize
void LidarCallback::Initialize() {
    if (udp_profile_lidar_ == "RNG19_RFL8_SIG16_NIR16") {
        PACKET_HEADER_BYTES = 32;
        PACKET_FOOTER_BYTES = 32;
        COLUMN_HEADER_BYTES = 12;
        CHANNEL_STRIDE_BYTES = 12;
        MEASUREMENT_BLOCK_STATUS_BYTES = 0;
    } else if (udp_profile_lidar_ == "LEGACY") {
        PACKET_HEADER_BYTES = 0;
        PACKET_FOOTER_BYTES = 0;
        COLUMN_HEADER_BYTES = 16;
        CHANNEL_STRIDE_BYTES = 12;
        MEASUREMENT_BLOCK_STATUS_BYTES = 4;
    } else {
        throw std::runtime_error("Unsupported udp_profile_lidar: " + udp_profile_lidar_);
    }

    block_size_ = COLUMN_HEADER_BYTES + (pixels_per_column_ * CHANNEL_STRIDE_BYTES) + MEASUREMENT_BLOCK_STATUS_BYTES;
    expected_size_ = PACKET_HEADER_BYTES + (columns_per_packet_ * block_size_) + PACKET_FOOTER_BYTES;

    const auto& beam_intrinsics = metadata_["beam_intrinsics"];
    const auto& azimuth_angles_json = beam_intrinsics["beam_azimuth_angles"];
    const auto& altitude_angles_json = beam_intrinsics["beam_altitude_angles"];

    if (!azimuth_angles_json.is_array() || azimuth_angles_json.size() != static_cast<size_t>(pixels_per_column_) ||
        !altitude_angles_json.is_array() || altitude_angles_json.size() != static_cast<size_t>(pixels_per_column_)) {
        throw std::runtime_error("Beam azimuth/altitude angles missing or wrong size in JSON.");
    }

    // Calculate number of channels in subset
    subset_channels_ = (pixels_per_column_ + channel_stride_ - 1) / channel_stride_; // Ceiling division

    // Reserve buffer space
    data_buffer1_.reserve(columns_per_frame_ * pixels_per_column_);
    data_buffer2_.reserve(columns_per_frame_ * pixels_per_column_);

    // Initialize original lookup tables
    sin_beam_azimuths_.resize(pixels_per_column_);
    cos_beam_azimuths_.resize(pixels_per_column_);
    sin_beam_altitudes_.resize(pixels_per_column_);
    cos_beam_altitudes_.resize(pixels_per_column_);
    r_min_.resize(pixels_per_column_, 5.0);
    x_1_.assign(columns_per_frame_, std::vector<double, Eigen::aligned_allocator<double>>(pixels_per_column_));
    y_1_.assign(columns_per_frame_, std::vector<double, Eigen::aligned_allocator<double>>(pixels_per_column_));
    z_1_.assign(columns_per_frame_, std::vector<double, Eigen::aligned_allocator<double>>(pixels_per_column_));
    x_2_.resize(columns_per_frame_);
    y_2_.resize(columns_per_frame_);
    z_2_.resize(columns_per_frame_);
    pixel_shifts_.resize(pixels_per_column_);

    // Initialize subset lookup tables
    sin_beam_azimuths_subset_.resize(subset_channels_);
    cos_beam_azimuths_subset_.resize(subset_channels_);
    sin_beam_altitudes_subset_.resize(subset_channels_);
    cos_beam_altitudes_subset_.resize(subset_channels_);
    r_min_subset_.resize(subset_channels_, 5.0);
    x_1_subset_.assign(columns_per_frame_, std::vector<double, Eigen::aligned_allocator<double>>(subset_channels_));
    y_1_subset_.assign(columns_per_frame_, std::vector<double, Eigen::aligned_allocator<double>>(subset_channels_));
    z_1_subset_.assign(columns_per_frame_, std::vector<double, Eigen::aligned_allocator<double>>(subset_channels_));
    pixel_shifts_subset_.resize(subset_channels_);
    subset_c_ids_.resize(subset_channels_);

    Eigen::Matrix4d lidar_to_desired_transform = Eigen::Matrix4d::Identity();
    switch (transform_preset_) {
        case LidarPreset::BERLIN20250730:
            lidar_to_desired_transform.block<3,3>(0,0) << -1, 0, 0, 0, -1, 0, 0, 0, 1;
            lidar_to_desired_transform.block<3,1>(0,3) << -0.135, 0.0, -0.1243;
            break;
        case LidarPreset::GEHLSDORF20250410:
            lidar_to_desired_transform.block<3,3>(0,0) << 0.021814, -0.999762, 0, -0.999762, -0.021814, 0, 0, 0, -1;
            lidar_to_desired_transform.block<3,1>(0,3) << -0.0775, 0.0, -0.17;
            break;
        case LidarPreset::BERLIN20250901:
            lidar_to_desired_transform.block<3,3>(0,0) << -1, 0, 0, 0, -1, 0, 0, 0, 1;
            lidar_to_desired_transform.block<3,1>(0,3) << -0.135, 0.0, -0.1243;
            break;
        default:
            throw std::runtime_error("Unsupported LidarTransformPreset selected.");
    }

    // Populate original and subset lookup tables
    for (int i = 0; i < pixels_per_column_; ++i) {
        double az_deg = azimuth_angles_json[i].get<double>();
        double alt_deg = altitude_angles_json[i].get<double>();
        double az_rad = az_deg * M_PI / 180.0;
        double alt_rad = alt_deg * M_PI / 180.0;
        sin_beam_azimuths_[i] = std::sin(az_rad);
        cos_beam_azimuths_[i] = std::cos(az_rad);
        sin_beam_altitudes_[i] = std::sin(alt_rad);
        cos_beam_altitudes_[i] = std::cos(alt_rad);
        pixel_shifts_[i] = metadata_["lidar_data_format"]["pixel_shift_by_row"][i].get<int>();

        // Populate subset tables if i is a multiple of channel_stride_
        if (i % channel_stride_ == 0) {
            size_t subset_idx = i / channel_stride_;
            sin_beam_azimuths_subset_[subset_idx] = sin_beam_azimuths_[i];
            cos_beam_azimuths_subset_[subset_idx] = cos_beam_azimuths_[i];
            sin_beam_altitudes_subset_[subset_idx] = sin_beam_altitudes_[i];
            cos_beam_altitudes_subset_[subset_idx] = cos_beam_altitudes_[i];
            r_min_subset_[subset_idx] = r_min_[i];
            pixel_shifts_subset_[subset_idx] = pixel_shifts_[i];
            subset_c_ids_[subset_idx] = i;
        }
    }

    for (int m_id = 0; m_id < columns_per_frame_; ++m_id) {
        double measurement_azimuth_rad = m_id * 2.0 * M_PI / columns_per_frame_;
        double cos_meas_az = std::cos(measurement_azimuth_rad);
        double sin_meas_az = std::sin(measurement_azimuth_rad);

        Eigen::Vector4d offset_lidar_frame(
            lidar_origin_to_beam_origin_mm_ * 0.001 * cos_meas_az,
            lidar_origin_to_beam_origin_mm_ * 0.001 * sin_meas_az,
            0.0,
            1.0);

        Eigen::Vector4d offset_transformed = lidar_to_desired_transform * offset_lidar_frame;
        x_2_[m_id] = offset_transformed.x();
        y_2_[m_id] = offset_transformed.y();
        z_2_[m_id] = offset_transformed.z();

        for (int ch = 0; ch < pixels_per_column_; ++ch) {
            double beam_az_rad = azimuth_angles_json[ch].get<double>() * M_PI / 180.0;
            double total_az_rad = measurement_azimuth_rad + beam_az_rad;
            double cos_total_az = std::cos(total_az_rad);
            double sin_total_az = std::sin(total_az_rad);
            double cos_alt = cos_beam_altitudes_[ch];
            double sin_alt = sin_beam_altitudes_[ch];

            Eigen::Vector4d dir_lidar_frame(cos_alt * cos_total_az, cos_alt * sin_total_az, sin_alt, 0.0);
            Eigen::Vector4d dir_transformed = lidar_to_desired_transform * dir_lidar_frame;
            x_1_[m_id][ch] = dir_transformed.x();
            y_1_[m_id][ch] = dir_transformed.y();
            z_1_[m_id][ch] = dir_transformed.z();

            // Populate subset tables if ch is a multiple of channel_stride_
            if (ch % channel_stride_ == 0) {
                size_t subset_idx = ch / channel_stride_;
                x_1_subset_[m_id][subset_idx] = x_1_[m_id][ch];
                y_1_subset_[m_id][subset_idx] = y_1_[m_id][ch];
                z_1_subset_[m_id][subset_idx] = z_1_[m_id][ch];
            }
        }
    }

    // Sanity checks for lookup table sizes
    if (x_1_.size() != static_cast<size_t>(columns_per_frame_) || (!x_1_.empty() && x_1_[0].size() != static_cast<size_t>(pixels_per_column_))) {
        throw std::runtime_error("x_1_ lookup table size mismatch after initialization");
    }
    if (x_1_subset_.size() != static_cast<size_t>(columns_per_frame_) || (!x_1_subset_.empty() && x_1_subset_[0].size() != static_cast<size_t>(subset_channels_))) {
        throw std::runtime_error("x_1_subset_ lookup table size mismatch after initialization");
    }
    if (x_2_.size() != static_cast<size_t>(columns_per_frame_)) {
        throw std::runtime_error("x_2_ lookup table size mismatch after initialization");
    }
    if (pixel_shifts_.size() != static_cast<size_t>(pixels_per_column_)) {
        throw std::runtime_error("pixel_shifts_ size mismatch after initialization");
    }
    if (pixel_shifts_subset_.size() != static_cast<size_t>(subset_channels_)) {
        throw std::runtime_error("pixel_shifts_subset_ size mismatch after initialization");
    }

#ifdef DEBUG
    if (!x_1_.empty() && !x_1_[0].empty()) {
        assert(reinterpret_cast<uintptr_t>(x_1_[0].data()) % 32 == 0 && "x_1_[0].data() not 32-byte aligned!");
    }
    if (!x_1_subset_.empty() && !x_1_subset_[0].empty()) {
        assert(reinterpret_cast<uintptr_t>(x_1_subset_[0].data()) % 32 == 0 && "x_1_subset_[0].data() not 32-byte aligned!");
    }
#endif
}
// %            ... decode_packet_legacy
void LidarCallback::DecodePacketLegacy(const std::vector<uint8_t>& packet, LidarFrame& frame) {
    if (packet.size() != expected_size_) {
        std::cerr << "Invalid packet size: " << packet.size() << ", expected: " << expected_size_ << std::endl;
        return;
    }

    LidarFrame* p_current_write_buffer;
    if (buffer_toggle_) {
        p_current_write_buffer = &data_buffer2_;
    } else {
        p_current_write_buffer = &data_buffer1_;
    }

    double prev_frame_completed_latest_ts = 0.0;

    for (int col = 0; col < columns_per_packet_; ++col) {
        size_t block_offset = col * block_size_;

        uint64_t timestamp_ns_raw;
        std::memcpy(&timestamp_ns_raw, packet.data() + block_offset, sizeof(uint64_t));
        uint64_t timestamp_ns = le64toh(timestamp_ns_raw);
        double current_col_timestamp_s = static_cast<double>(timestamp_ns) * 1e-9;

        if (current_col_timestamp_s < 0) {
            std::cerr << "Negative column timestamp: " << current_col_timestamp_s << std::endl;
            continue;
        }

        uint16_t m_id_raw;
        std::memcpy(&m_id_raw, packet.data() + block_offset + 8, sizeof(uint16_t));
        uint16_t m_id = le16toh(m_id_raw);

        if (m_id >= static_cast<uint16_t>(columns_per_frame_)) {
            std::cerr << "Invalid measurement ID: " << m_id << " (>= " << columns_per_frame_ << ")" << std::endl;
            continue;
        }

        uint16_t current_packet_frame_id_raw;
        std::memcpy(&current_packet_frame_id_raw, packet.data() + block_offset + 10, sizeof(uint16_t));
        uint16_t current_packet_frame_id = le16toh(current_packet_frame_id_raw);

        if (current_packet_frame_id != this->frame_id_) {
            if (this->frame_id_ != 0 || this->number_points_ > 0) {
                p_current_write_buffer->numberpoints = this->number_points_;
                p_current_write_buffer->timestamp_end = this->latest_timestamp_s;
            }
            prev_frame_completed_latest_ts = this->latest_timestamp_s;
            SwapBuffer();
            if (buffer_toggle_) {
                p_current_write_buffer = &data_buffer2_;
            } else {
                p_current_write_buffer = &data_buffer1_;
            }
            this->number_points_ = 0;
            this->frame_id_ = current_packet_frame_id;
            p_current_write_buffer->clear();
            p_current_write_buffer->reserve(columns_per_frame_ * pixels_per_column_);
        }

        this->latest_timestamp_s = current_col_timestamp_s;

        uint32_t block_status;
        size_t status_offset = block_offset + 16 + (pixels_per_column_ * 12);
        std::memcpy(&block_status, packet.data() + status_offset, sizeof(uint32_t));
        block_status = le32toh(block_status);
        if (block_status != 0xFFFFFFFF) {
            continue;
        }

        bool is_first_point_of_current_frame = (this->number_points_ == 0);

#ifdef __AVX2__
        for (uint16_t subset_idx_base = 0; subset_idx_base < static_cast<uint16_t>(subset_channels_); subset_idx_base += 4) {
            size_t first_pixel_in_block_offset = block_offset + 16;

            alignas(32) double range_m[4];
            alignas(32) double r_min_vals[4];
            uint16_t c_ids[4];
            uint8_t reflectivity[4];
            uint16_t signal[4], nir[4];

            for (int i = 0; i < 4; ++i) {
                uint16_t subset_idx = subset_idx_base + i;
                if (subset_idx >= static_cast<uint16_t>(subset_channels_)) {
                    range_m[i] = 0.0;
                    r_min_vals[i] = 1.0;
                    c_ids[i] = 0;
                    continue;
                }

                uint16_t current_c_id = subset_c_ids_[subset_idx];
                size_t pixel_data_offset = first_pixel_in_block_offset + current_c_id * 12;
                if (pixel_data_offset + 11 >= packet.size()) {
                    range_m[i] = 0.0;
                    r_min_vals[i] = 1.0;
                    c_ids[i] = 0;
                    continue;
                }

                uint32_t range_mm_raw;
                std::memcpy(&range_mm_raw, packet.data() + pixel_data_offset, sizeof(uint32_t));
                uint32_t range_mm = le32toh(range_mm_raw) & 0x000FFFFF;
                range_m[i] = static_cast<double>(range_mm) * 0.001;
                r_min_vals[i] = r_min_subset_[subset_idx];
                c_ids[i] = current_c_id;

                std::memcpy(&reflectivity[i], packet.data() + pixel_data_offset + 4, sizeof(uint8_t));
                uint16_t signal_raw, nir_raw;
                std::memcpy(&signal_raw, packet.data() + pixel_data_offset + 6, sizeof(uint16_t));
                std::memcpy(&nir_raw, packet.data() + pixel_data_offset + 8, sizeof(uint16_t));
                signal[i] = le16toh(signal_raw);
                nir[i] = le16toh(nir_raw);
            }

            __m256d m256d_range = _mm256_load_pd(range_m);
            __m256d m256d_r_min_vec = _mm256_load_pd(r_min_vals);
            __m256d valid_mask = _mm256_cmp_pd(m256d_range, m256d_r_min_vec, _CMP_GE_OQ);

            __m256d x1_vec = _mm256_load_pd(x_1_subset_[m_id].data() + subset_idx_base);
            __m256d y1_vec = _mm256_load_pd(y_1_subset_[m_id].data() + subset_idx_base);
            __m256d z1_vec = _mm256_load_pd(z_1_subset_[m_id].data() + subset_idx_base);
            __m256d x2_val = _mm256_set1_pd(x_2_[m_id]);
            __m256d y2_val = _mm256_set1_pd(y_2_[m_id]);
            __m256d z2_val = _mm256_set1_pd(z_2_[m_id]);

            __m256d pt_x = _mm256_fmadd_pd(m256d_range, x1_vec, x2_val);
            __m256d pt_y = _mm256_fmadd_pd(m256d_range, y1_vec, y2_val);
            __m256d pt_z = _mm256_fmadd_pd(m256d_range, z1_vec, z2_val);

            alignas(32) double pt_x_arr[4], pt_y_arr[4], pt_z_arr[4];
            _mm256_store_pd(pt_x_arr, pt_x);
            _mm256_store_pd(pt_y_arr, pt_y);
            _mm256_store_pd(pt_z_arr, pt_z);

            alignas(32) double valid_mask_arr[4];
            _mm256_store_pd(valid_mask_arr, valid_mask);

            double relative_timestamp_s = (p_current_write_buffer->numberpoints > 0 || this->number_points_ > 0) && p_current_write_buffer->timestamp > 0
                ? std::max(0.0, current_col_timestamp_s - p_current_write_buffer->timestamp)
                : 0.0;

            for (int i = 0; i < 4; ++i) {
                uint16_t subset_idx = subset_idx_base + i;
                if (subset_idx >= static_cast<uint16_t>(subset_channels_)) break;
                if (range_m[i] >= r_min_vals[i] && range_m[i] > 0) {
                    p_current_write_buffer->x.push_back(pt_x_arr[i]);
                    p_current_write_buffer->y.push_back(pt_y_arr[i]);
                    p_current_write_buffer->z.push_back(pt_z_arr[i]);
                    p_current_write_buffer->c_id.push_back(c_ids[i]);
                    p_current_write_buffer->m_id.push_back(m_id);
                    p_current_write_buffer->timestamp_points.push_back(current_col_timestamp_s);
                    p_current_write_buffer->relative_timestamp.push_back(relative_timestamp_s);
                    p_current_write_buffer->reflectivity.push_back(reflectivity[i]);
                    p_current_write_buffer->signal.push_back(signal[i]);
                    p_current_write_buffer->nir.push_back(nir[i]);

                    this->number_points_++;
                    if (is_first_point_of_current_frame) {
                        p_current_write_buffer->timestamp = current_col_timestamp_s;
                        p_current_write_buffer->frame_id = this->frame_id_;
                        p_current_write_buffer->interframe_timedelta = (prev_frame_completed_latest_ts > 0.0)
                            ? std::max(0.0, current_col_timestamp_s - prev_frame_completed_latest_ts) : 0.0;
                        is_first_point_of_current_frame = false;
                    }
                }
            }
        }
#else
        // Scalar fallback remains unchanged, using original tables
        size_t first_pixel_in_block_offset = block_offset + 16;
        for (uint16_t c_id = 0; c_id < static_cast<uint16_t>(pixels_per_column_); c_id += channel_stride_) {
            size_t pixel_data_offset = first_pixel_in_block_offset + c_id * 12;
            if (pixel_data_offset + 11 >= packet.size()) {
                continue;
            }

            uint32_t range_mm_raw;
            std::memcpy(&range_mm_raw, packet.data() + pixel_data_offset, sizeof(uint32_t));
            uint32_t range_mm = le32toh(range_mm_raw) & 0x000FFFFF;
            double range_m = static_cast<double>(range_mm) * 0.001;

            if (range_m < r_min_[c_id] || range_m == 0) {
                continue;
            }

            uint8_t current_reflectivity;
            std::memcpy(&current_reflectivity, packet.data() + pixel_data_offset + 4, sizeof(uint8_t));
            uint16_t signal_raw, nir_raw;
            std::memcpy(&signal_raw, packet.data() + pixel_data_offset + 6, sizeof(uint16_t));
            std::memcpy(&nir_raw, packet.data() + pixel_data_offset + 8, sizeof(uint16_t));
            uint16_t current_signal = le16toh(signal_raw);
            uint16_t current_nir = le16toh(nir_raw);

            double pt_x = range_m * x_1_[m_id][c_id] + x_2_[m_id];
            double pt_y = range_m * y_1_[m_id][c_id] + y_2_[m_id];
            double pt_z = range_m * z_1_[m_id][c_id] + z_2_[m_id];

            double relative_timestamp_s = (p_current_write_buffer->numberpoints > 0 || this->number_points_ > 0) && p_current_write_buffer->timestamp > 0
                ? std::max(0.0, current_col_timestamp_s - p_current_write_buffer->timestamp)
                : 0.0;

            p_current_write_buffer->x.push_back(pt_x);
            p_current_write_buffer->y.push_back(pt_y);
            p_current_write_buffer->z.push_back(pt_z);
            p_current_write_buffer->c_id.push_back(c_id);
            p_current_write_buffer->m_id.push_back(m_id);
            p_current_write_buffer->timestamp_points.push_back(current_col_timestamp_s);
            p_current_write_buffer->relative_timestamp.push_back(relative_timestamp_s);
            p_current_write_buffer->reflectivity.push_back(current_reflectivity);
            p_current_write_buffer->signal.push_back(current_signal);
            p_current_write_buffer->nir.push_back(current_nir);

            this->number_points_++;
            if (is_first_point_of_current_frame) {
                p_current_write_buffer->timestamp = current_col_timestamp_s;
                p_current_write_buffer->frame_id = this->frame_id_;
                p_current_write_buffer->interframe_timedelta = (prev_frame_completed_latest_ts > 0.0)
                    ? std::max(0.0, current_col_timestamp_s - prev_frame_completed_latest_ts) : 0.0;
                is_first_point_of_current_frame = false;
            }
        }
#endif
    }

    if (p_current_write_buffer) {
        p_current_write_buffer->numberpoints = this->number_points_;
    }
    frame = GetLatestFrame();
}
// %            ... decode_packet_single_return
void LidarCallback::DecodePacketRng19(const std::vector<uint8_t>& packet, LidarFrame& frame) {
    if (packet.size() != expected_size_) {
        std::cerr << "Invalid packet size: " << packet.size() << ", expected: " << expected_size_ << std::endl;
        return;
    }

    uint16_t packet_type_raw;
    std::memcpy(&packet_type_raw, packet.data(), sizeof(uint16_t));
    uint16_t packet_type = le16toh(packet_type_raw);
    if (packet_type != 0x0001) {
        std::cerr << "Invalid packet type: 0x" << std::hex << packet_type << std::dec << " (expected 0x1)" << std::endl;
        return;
    }

    uint16_t current_packet_frame_id_raw;
    std::memcpy(&current_packet_frame_id_raw, packet.data() + 2, sizeof(uint16_t));
    uint16_t current_packet_frame_id = le16toh(current_packet_frame_id_raw);

    LidarFrame* p_current_write_buffer;
    if (buffer_toggle_) {
        p_current_write_buffer = &data_buffer2_;
    } else {
        p_current_write_buffer = &data_buffer1_;
    }

    double prev_frame_completed_latest_ts = 0.0;

    if (current_packet_frame_id != this->frame_id_) {
        if (this->frame_id_ != 0 || this->number_points_ > 0) {
            p_current_write_buffer->numberpoints = this->number_points_;
            p_current_write_buffer->timestamp_end = this->latest_timestamp_s;
        }
        prev_frame_completed_latest_ts = this->latest_timestamp_s;
        SwapBuffer();
        if (buffer_toggle_) {
            p_current_write_buffer = &data_buffer2_;
        } else {
            p_current_write_buffer = &data_buffer1_;
        }
        this->number_points_ = 0;
        this->frame_id_ = current_packet_frame_id;
        p_current_write_buffer->clear();
        p_current_write_buffer->reserve(columns_per_frame_ * pixels_per_column_);
    }

    bool is_first_point_of_current_frame = (this->number_points_ == 0);

    for (int col = 0; col < columns_per_packet_; ++col) {
        size_t block_offset = PACKET_HEADER_BYTES + col * block_size_;

        uint64_t timestamp_ns_raw;
        std::memcpy(&timestamp_ns_raw, packet.data() + block_offset, sizeof(uint64_t));
        uint64_t timestamp_ns = le64toh(timestamp_ns_raw);
        double current_col_timestamp_s = static_cast<double>(timestamp_ns) * 1e-9;
        
        if (current_col_timestamp_s < 0) {
            std::cerr << "Negative column timestamp: " << current_col_timestamp_s << std::endl;
            continue;
        }
        this->latest_timestamp_s = current_col_timestamp_s;

        uint16_t m_id_raw;
        std::memcpy(&m_id_raw, packet.data() + block_offset + 8, sizeof(uint16_t));
        uint16_t m_id = le16toh(m_id_raw);

        if (m_id >= static_cast<uint16_t>(columns_per_frame_)) {
            std::cerr << "Invalid measurement ID: " << m_id << " (>= " << columns_per_frame_ << ")" << std::endl;
            continue;
        }

        uint8_t column_status;
        std::memcpy(&column_status, packet.data() + block_offset + 10, sizeof(uint8_t));
        if (!(column_status & 0x01)) {
            continue;
        }

#ifdef __AVX2__
        for (uint16_t subset_idx_base = 0; subset_idx_base < static_cast<uint16_t>(subset_channels_); subset_idx_base += 4) {
            size_t first_pixel_in_block_offset = block_offset + 12;

            alignas(32) double range_m[4];
            alignas(32) double r_min_vals[4];
            uint16_t c_ids[4];
            uint8_t reflectivity[4];
            uint16_t signal[4], nir[4];

            for (int i = 0; i < 4; ++i) {
                uint16_t subset_idx = subset_idx_base + i;
                if (subset_idx >= static_cast<uint16_t>(subset_channels_)) {
                    range_m[i] = 0.0;
                    r_min_vals[i] = 1.0;
                    c_ids[i] = 0;
                    continue;
                }

                uint16_t current_c_id = subset_c_ids_[subset_idx];
                size_t pixel_data_offset = first_pixel_in_block_offset + current_c_id * 12;
                if (pixel_data_offset + 11 >= packet.size()) {
                    range_m[i] = 0.0;
                    r_min_vals[i] = 1.0;
                    c_ids[i] = 0;
                    continue;
                }

                uint32_t range_mm_raw;
                uint8_t range_bytes[4] = {packet[pixel_data_offset], packet[pixel_data_offset + 1], packet[pixel_data_offset + 2], 0};
                std::memcpy(&range_mm_raw, range_bytes, sizeof(uint32_t));
                uint32_t range_mm = le32toh(range_mm_raw) & 0x0007FFFF;
                range_m[i] = static_cast<double>(range_mm) * 0.001;
                r_min_vals[i] = r_min_subset_[subset_idx];
                c_ids[i] = current_c_id;

                std::memcpy(&reflectivity[i], packet.data() + pixel_data_offset + 4, sizeof(uint8_t));
                uint16_t signal_raw, nir_raw;
                std::memcpy(&signal_raw, packet.data() + pixel_data_offset + 6, sizeof(uint16_t));
                std::memcpy(&nir_raw, packet.data() + pixel_data_offset + 8, sizeof(uint16_t));
                signal[i] = le16toh(signal_raw);
                nir[i] = le16toh(nir_raw);
            }

            __m256d m256d_range = _mm256_load_pd(range_m);
            __m256d m256d_r_min_vec = _mm256_load_pd(r_min_vals);
            __m256d valid_mask = _mm256_cmp_pd(m256d_range, m256d_r_min_vec, _CMP_GE_OQ);

            __m256d x1_vec = _mm256_load_pd(x_1_subset_[m_id].data() + subset_idx_base);
            __m256d y1_vec = _mm256_load_pd(y_1_subset_[m_id].data() + subset_idx_base);
            __m256d z1_vec = _mm256_load_pd(z_1_subset_[m_id].data() + subset_idx_base);
            __m256d x2_val = _mm256_set1_pd(x_2_[m_id]);
            __m256d y2_val = _mm256_set1_pd(y_2_[m_id]);
            __m256d z2_val = _mm256_set1_pd(z_2_[m_id]);

            __m256d pt_x = _mm256_fmadd_pd(m256d_range, x1_vec, x2_val);
            __m256d pt_y = _mm256_fmadd_pd(m256d_range, y1_vec, y2_val);
            __m256d pt_z = _mm256_fmadd_pd(m256d_range, z1_vec, z2_val);

            alignas(32) double pt_x_arr[4], pt_y_arr[4], pt_z_arr[4];
            _mm256_store_pd(pt_x_arr, pt_x);
            _mm256_store_pd(pt_y_arr, pt_y);
            _mm256_store_pd(pt_z_arr, pt_z);

            alignas(32) double valid_mask_arr[4];
            _mm256_store_pd(valid_mask_arr, valid_mask);

            double relative_timestamp_s = (p_current_write_buffer->numberpoints > 0 || this->number_points_ > 0) && p_current_write_buffer->timestamp > 0
                ? std::max(0.0, current_col_timestamp_s - p_current_write_buffer->timestamp)
                : 0.0;

            for (int i = 0; i < 4; ++i) {
                uint16_t subset_idx = subset_idx_base + i;
                if (subset_idx >= static_cast<uint16_t>(subset_channels_)) break;
                if (range_m[i] >= r_min_vals[i] && range_m[i] > 0) {
                    p_current_write_buffer->x.push_back(pt_x_arr[i]);
                    p_current_write_buffer->y.push_back(pt_y_arr[i]);
                    p_current_write_buffer->z.push_back(pt_z_arr[i]);
                    p_current_write_buffer->c_id.push_back(c_ids[i]);
                    p_current_write_buffer->m_id.push_back(m_id);
                    p_current_write_buffer->timestamp_points.push_back(current_col_timestamp_s);
                    p_current_write_buffer->relative_timestamp.push_back(relative_timestamp_s);
                    p_current_write_buffer->reflectivity.push_back(reflectivity[i]);
                    p_current_write_buffer->signal.push_back(signal[i]);
                    p_current_write_buffer->nir.push_back(nir[i]);

                    this->number_points_++;
                    if (is_first_point_of_current_frame) {
                        p_current_write_buffer->timestamp = current_col_timestamp_s;
                        p_current_write_buffer->frame_id = this->frame_id_;
                        p_current_write_buffer->interframe_timedelta = (prev_frame_completed_latest_ts > 0.0)
                            ? std::max(0.0, current_col_timestamp_s - prev_frame_completed_latest_ts) : 0.0;
                        is_first_point_of_current_frame = false;
                    }
                }
            }
        }
#else
        // Scalar fallback remains unchanged, using original tables
        size_t first_pixel_in_block_offset = block_offset + 12;
        for (uint16_t c_id = 0; c_id < static_cast<uint16_t>(pixels_per_column_); c_id += channel_stride_) {
            size_t pixel_data_offset = first_pixel_in_block_offset + c_id * 12;
            if (pixel_data_offset + 11 >= packet.size()) {
                continue;
            }

            uint32_t range_mm_raw;
            uint8_t range_bytes[4] = {packet[pixel_data_offset], packet[pixel_data_offset + 1], packet[pixel_data_offset + 2], 0};
            std::memcpy(&range_mm_raw, range_bytes, sizeof(uint32_t));
            uint32_t range_mm = le32toh(range_mm_raw) & 0x0007FFFF;
            double range_m = static_cast<double>(range_mm) * 0.001;

            if (range_m < r_min_[c_id] || range_m == 0) {
                continue;
            }

            uint8_t current_reflectivity;
            std::memcpy(&current_reflectivity, packet.data() + pixel_data_offset + 4, sizeof(uint8_t));
            uint16_t signal_raw, nir_raw;
            std::memcpy(&signal_raw, packet.data() + pixel_data_offset + 6, sizeof(uint16_t));
            std::memcpy(&nir_raw, packet.data() + pixel_data_offset + 8, sizeof(uint16_t));
            uint16_t current_signal = le16toh(signal_raw);
            uint16_t current_nir = le16toh(nir_raw);

            double pt_x = range_m * x_1_[m_id][c_id] + x_2_[m_id];
            double pt_y = range_m * y_1_[m_id][c_id] + y_2_[m_id];
            double pt_z = range_m * z_1_[m_id][c_id] + z_2_[m_id];

            double relative_timestamp_s = (p_current_write_buffer->numberpoints > 0 || this->number_points_ > 0) && p_current_write_buffer->timestamp > 0
                ? std::max(0.0, current_col_timestamp_s - p_current_write_buffer->timestamp)
                : 0.0;

            p_current_write_buffer->x.push_back(pt_x);
            p_current_write_buffer->y.push_back(pt_y);
            p_current_write_buffer->z.push_back(pt_z);
            p_current_write_buffer->c_id.push_back(c_id);
            p_current_write_buffer->m_id.push_back(m_id);
            p_current_write_buffer->timestamp_points.push_back(current_col_timestamp_s);
            p_current_write_buffer->relative_timestamp.push_back(relative_timestamp_s);
            p_current_write_buffer->reflectivity.push_back(current_reflectivity);
            p_current_write_buffer->signal.push_back(current_signal);
            p_current_write_buffer->nir.push_back(current_nir);

            this->number_points_++;
            if (is_first_point_of_current_frame) {
                p_current_write_buffer->timestamp = current_col_timestamp_s;
                p_current_write_buffer->frame_id = this->frame_id_;
                p_current_write_buffer->interframe_timedelta = (prev_frame_completed_latest_ts > 0.0)
                    ? std::max(0.0, current_col_timestamp_s - prev_frame_completed_latest_ts) : 0.0;
                is_first_point_of_current_frame = false;
            }
        }
#endif
    }

    if (p_current_write_buffer) {
        p_current_write_buffer->numberpoints = this->number_points_;
    }
    frame = GetLatestFrame();
}