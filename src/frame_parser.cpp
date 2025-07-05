#include "sipeed_tof_ms_a010/frame_parser.hpp"
#include <algorithm>
#include <cstring> // For memcpy

namespace sipeed_tof
{

FrameParser::FrameParser() = default;

void FrameParser::add_data(const std::string& new_data)
{
    buffer_.insert(buffer_.end(), new_data.begin(), new_data.end());
}

FramePtr FrameParser::parse()
{
    const uint8_t sflag_l = FRAME_BEGIN_FLAG & 0xff;
    const uint8_t sflag_h = (FRAME_BEGIN_FLAG >> 8) & 0xff;
    const uint8_t eflag = FRAME_END_FLAG & 0xff;

    while (buffer_.size() >= 2)
    {
        // Find the start of a potential frame
        auto it = std::find(buffer_.begin(), buffer_.end(), sflag_l);
        if (it == buffer_.end() || (it + 1) == buffer_.end()) {
            buffer_.clear(); // No start flag found, or it's the last byte
            return nullptr;
        }

        // Erase all data before the potential start flag
        if (it != buffer_.begin()) {
            buffer_.erase(buffer_.begin(), it);
        }

        // Check if the next byte is the high byte of the start flag
        if (*(buffer_.begin() + 1) != sflag_h) {
            // False start, remove the low byte and try again
            buffer_.erase(buffer_.begin());
            continue;
        }

        // We have a potential frame header
        if (buffer_.size() < sizeof(frame_head_t)) {
            return nullptr; // Not enough data for a full header yet
        }

        auto* pf_head = reinterpret_cast<frame_head_t*>(buffer_.data());
        uint32_t frame_payload_len = pf_head->frame_data_len - FRAME_HEAD_DATA_SIZE;
        uint32_t total_frame_size = FRAME_HEAD_SIZE + frame_payload_len + FRAME_CHECKSUM_SIZE + FRAME_END_SIZE;

        // Sanity check on payload length
        if (frame_payload_len > 20000) { // Max possible size for 100*100*2
            buffer_.erase(buffer_.begin(), buffer_.begin() + 2); // Corrupted length, skip header
            continue;
        }

        if (buffer_.size() < total_frame_size) {
            return nullptr; // Not enough data for the full frame yet
        }

        // Verify checksum
        uint8_t check_sum = 0;
        for (uint32_t i = 0; i < FRAME_HEAD_SIZE + frame_payload_len; i++) {
            check_sum += buffer_[i];
        }

        uint8_t frame_checksum = buffer_[FRAME_HEAD_SIZE + frame_payload_len];
        uint8_t frame_end_flag = buffer_[FRAME_HEAD_SIZE + frame_payload_len + FRAME_CHECKSUM_SIZE];

        if (check_sum == frame_checksum && eflag == frame_end_flag) {
            // Valid frame found!
            void* mem = malloc(total_frame_size);
            if (!mem) {
                // Allocation failed, discard and try to recover
                buffer_.erase(buffer_.begin(), buffer_.begin() + total_frame_size);
                continue;
            }
            memcpy(mem, buffer_.data(), total_frame_size);
            
            // Remove the processed frame from the buffer
            buffer_.erase(buffer_.begin(), buffer_.begin() + total_frame_size);

            return FramePtr(static_cast<frame_t*>(mem));
        } else {
            // Invalid frame, discard the header and search again
            buffer_.erase(buffer_.begin(), buffer_.begin() + 2);
        }
    }

    return nullptr; // No complete frame found
}

} // namespace sipeed_tof