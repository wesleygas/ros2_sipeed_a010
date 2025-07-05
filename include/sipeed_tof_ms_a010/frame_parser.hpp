#pragma once

#include "frame_struct.h"
#include <vector>
#include <string>
#include <memory>

namespace sipeed_tof
{

// Custom deleter for the unique_ptr to use free()
struct FrameDeleter {
    void operator()(frame_t* p) const {
        free(p);
    }
};

using FramePtr = std::unique_ptr<frame_t, FrameDeleter>;

class FrameParser
{
public:
  FrameParser();
  void add_data(const std::string& new_data);
  FramePtr parse();

private:
  std::vector<uint8_t> buffer_;
  // A safety limit to prevent the buffer from growing indefinitely with corrupt data.
  // 3 * (max frame size) should be plenty. 100x100 resolution -> ~10KB frame.
  static constexpr size_t MAX_BUFFER_SIZE = 3 * 12000;
};

} // namespace sipeed_tof