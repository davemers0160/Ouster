#ifndef _OUSTER_PACKET_H
#define _OUSTER_PACKET_H


#include <cstdint>

const uint32_t packet_size = 12608;
const int pixels_per_column = 64;
const int columns_per_buffer = 16;

const int pixel_bytes = 12;
const int column_bytes = 16 + (pixels_per_column * pixel_bytes) + 4;

const int encoder_ticks_per_rev = 90112;

inline const uint8_t* nth_col(int n, const uint8_t* udp_buf) {
    return udp_buf + (n * column_bytes);
}

inline uint64_t col_timestamp(const uint8_t* col_buf) {
    uint64_t res;
    memcpy(&res, col_buf, sizeof(uint64_t));
    return res;
}

inline uint32_t get_measurement_id(const uint8_t* buf) {
    uint32_t measure_id;
    memcpy(&measure_id, buf + 8, sizeof(uint32_t));
    return measure_id;
}

inline uint32_t get_encoder_count(const uint8_t* buf) {
    uint32_t encoder;
    memcpy(&encoder, buf + 12, sizeof(uint32_t));
    return encoder;
}

//-----------------------------------------------------------------------------
//  Lidar Pixel Fields
//-----------------------------------------------------------------------------
inline const uint8_t* nth_px(int n, const uint8_t* col_buf) {
    return col_buf + 16 + (n * pixel_bytes);
}

inline uint32_t px_range(const uint8_t* px_buf) {
    uint32_t res;
    memcpy(&res, px_buf, sizeof(uint32_t));
    res &= 0x000fffff;
    return res;
}

inline uint16_t px_reflectivity(const uint8_t* px_buf) {
    uint16_t res;
    memcpy(&res, px_buf + 4, sizeof(uint16_t));
    return res;
}

inline uint16_t px_signal_photons(const uint8_t* px_buf) {
    uint16_t res;
    memcpy(&res, px_buf + 6, sizeof(uint16_t));
    return res;
}


#endif  //_OUSTER_PACKET_H