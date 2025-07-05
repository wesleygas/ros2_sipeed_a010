#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define FRAME_BEGIN_FLAG (0xFF00)
#define FRAME_END_FLAG (0xDD)

#define FRAME_HEAD_SIZE (20)
#define FRAME_HEAD_DATA_SIZE (16)
#define FRAME_CHECKSUM_SIZE (1)
#define FRAME_END_SIZE (1)

typedef struct {
  uint16_t frame_begin_flag;
  uint16_t frame_data_len;
  uint8_t reserved1;
  uint8_t output_mode;
  uint8_t senser_temp;
  uint8_t driver_temp;
  uint8_t exposure_time[4];
  uint8_t error_code;
  uint8_t reserved2;
  uint8_t resolution_rows;
  uint8_t resolution_cols;
  uint16_t frame_id;
  uint8_t isp_version;
  uint8_t reserved3;
} __attribute__((packed)) frame_head_t;

typedef struct {
  frame_head_t frame_head;
  uint8_t payload[1];
} __attribute__((packed)) frame_t;

#ifdef __cplusplus
}
#endif