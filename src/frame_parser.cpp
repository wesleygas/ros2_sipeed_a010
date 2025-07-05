#include "sipeed_tof_ms_a010/frame_parser.hpp"
#include <algorithm>
#include <cstring> // For memcpy

namespace sipeed_tof
{

FrameParser::FrameParser() = default;

void FrameParser::add_data(const std::string& new_data)
{
    // If the buffer is getting too large, clear it to resynchronize.
    // This prevents uncontrolled memory growth from corrupt data streams.
    if (buffer_.size() > MAX_BUFFER_SIZE) {
        buffer_.clear();
    }
    buffer_.insert(buffer_.end(), new_data.begin(), new_data.end());
}

FramePtr FrameParser::parse()
{
    const uint8_t sflag_l = FRAME_BEGIN_FLAG & 0xff;
    const uint8_t sflag_h = (FRAME_BEGIN_FLAG >> 8) & 0xff;
    const uint8_t eflag = FRAME_END_FLAG & 0xff;

    while (true)
    {
        // 1. Find the beginning of a potential frame.
        auto it = std::find(buffer_.begin(), buffer_.end(), sflag_l);

        // If no start flag, discard all but the last byte (which could be the start of a future flag)
        if (it == buffer_.end()) {
            if (buffer_.size() > 0) {
                buffer_.erase(buffer_.begin(), buffer_.end() - 1);
            }
            return nullptr;
        }

        // Discard all bytes before the potential start flag.
        buffer_.erase(buffer_.begin(), it);

        // 2. Check if we have a full header.
        if (buffer_.size() < sizeof(frame_head_t)) {
            return nullptr; // Not enough data yet, wait for more.
        }

        // 3. Validate the header.
        if (buffer_[1] != sflag_h) {
            // It was a false start (0xFF was not followed by 0x00). Discard the 0xFF and loop again.
            buffer_.erase(buffer_.begin());
            continue; // Go back to step 1
        }

        // Header looks plausible (starts with 0xFF00).
        auto* pf_head = reinterpret_cast<frame_head_t*>(buffer_.data());

        // 4. Validate the length from the header.
        if (pf_head->frame_data_len < FRAME_HEAD_DATA_SIZE) {
            // Corrupt length. Discard the bad header and loop again.
            buffer_.erase(buffer_.begin(), buffer_.begin() + 2);
            continue; // Go back to step 1
        }
        
        uint32_t frame_payload_len = pf_head->frame_data_len - FRAME_HEAD_DATA_SIZE;
        uint32_t total_frame_size = FRAME_HEAD_SIZE + frame_payload_len + FRAME_CHECKSUM_SIZE + FRAME_END_SIZE;

        if (frame_payload_len > 12000) { // Sanity check
            buffer_.erase(buffer_.begin(), buffer_.begin() + 2);
            continue; // Go back to step 1
        }

        // 5. Check if the full frame is in the buffer.
        if (buffer_.size() < total_frame_size) {
            return nullptr; // Not enough data yet, wait for more.
        }

        // 6. Verify checksum and end flag.
        uint8_t calculated_checksum = 0;
        for (uint32_t j = 0; j < FRAME_HEAD_SIZE + frame_payload_len; j++) {
            calculated_checksum += buffer_[j];
        }

        uint8_t frame_checksum = buffer_[FRAME_HEAD_SIZE + frame_payload_len];
        uint8_t frame_end_flag = buffer_[FRAME_HEAD_SIZE + frame_payload_len + FRAME_CHECKSUM_SIZE];

        if (calculated_checksum == frame_checksum && eflag == frame_end_flag)
        {
            // SUCCESS! We have a valid frame.
            void* mem = malloc(total_frame_size);
            if (!mem) {
                buffer_.clear(); // Out of memory, panic and clear.
                return nullptr;
            }
            memcpy(mem, buffer_.data(), total_frame_size);
            
            // Remove the frame from the buffer and return it.
            buffer_.erase(buffer_.begin(), buffer_.begin() + total_frame_size);
            return FramePtr(static_cast<frame_t*>(mem));
        }
        else
        {
            // Checksum failed. Discard the bad header and loop again.
            buffer_.erase(buffer_.begin(), buffer_.begin() + 2);
            continue; // Go back to step 1
        }
    }
}

} // namespace sipeed_tof