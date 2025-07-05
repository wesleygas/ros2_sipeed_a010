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
};

} // namespace sipeed_tof