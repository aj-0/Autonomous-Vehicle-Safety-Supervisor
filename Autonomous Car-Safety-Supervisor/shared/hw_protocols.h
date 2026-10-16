/*
 * hw_protocols.h - pure (hardware-free) parts of the real-rover drivers, unit-tested on the PC:
 *
 *   TF-Luna LiDAR   : 9-byte UART frame parser
 *   wheel encoders  : speed from pulse timestamps
 *   Jetson camera   : UDP detection packet check (layout below, little-endian)
 */
#ifndef HW_PROTOCOLS_H
#define HW_PROTOCOLS_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/* TF-Luna (UART mode, 115200 8N1, 100 frames/s)                       */
/*   0x59 0x59 DistL DistH AmpL AmpH TempL TempH Checksum              */
/*   distance in cm, checksum = low byte of the sum of bytes 0..7      */
/* ------------------------------------------------------------------ */
typedef struct {
    uint8_t buf[9];
    int     n;
    uint32_t frames, bad_checksum;
} tfluna_parser_t;

typedef struct {
    float    dist_m;
    uint16_t amp;               /* signal strength: < 100 or 0xFFFF = unreliable */
    float    temp_c;
} tfluna_frame_t;

void tfluna_init(tfluna_parser_t *p);
/* feed one byte; returns 1 when a complete, checksum-valid frame was decoded into *out */
int  tfluna_feed(tfluna_parser_t *p, uint8_t byte, tfluna_frame_t *out);

/* ------------------------------------------------------------------ */
/* Wheel encoder (single-channel slotted disc)                        */
/* ------------------------------------------------------------------ */
#define ENC_HIST 16

typedef struct {
    uint64_t t[ENC_HIST];       /* timestamps of the last pulses (us) */
    int      head, count;
    uint32_t total;             /* all accepted pulses                */
    float    v;                 /* filtered speed, m/s                */
    uint64_t last_update_us;
} enc_est_t;

void  enc_init(enc_est_t *e);
/* one slot edge at time t_us; edges closer than min_gap_us are bounce and ignored */
void  enc_pulse(enc_est_t *e, uint64_t t_us, uint32_t min_gap_us);
/* speed now: average over the pulses of the last window_us, decays when pulses stop,
 * change limited to max_accel (m/s^2) - a wheel cannot change speed faster than that */
float enc_speed(enc_est_t *e, uint64_t now_us, float m_per_pulse, uint32_t window_us,
                float max_accel);

/* ------------------------------------------------------------------ */
/* Jetson -> Pi camera detection packet, UDP, 24 bytes, little-endian  */
/*                                                                     */
/*  off size field                                                     */
/*   0   4   magic       0x314A5641 ("AVJ1")                           */
/*   4   2   version     1                                             */
/*   6   2   length      24                                            */
/*   8   4   seq         +1 per camera frame                           */
/*  12   1   obstacle    0 / 1                                         */
/*  13   1   zone        0 none, 1 left, 2 center, 3 right              */
/*  14   1   confidence  0..100 %                                      */
/*  15   1   class_id    COCO class of the object                      */
/*  16   4   dist_m      float, estimated distance, -1 = unknown        */
/*  20   2   age_ms      camera capture -> packet sent, on the Jetson   */
/*  22   2   crc16       CRC-16/CCITT-FALSE of bytes 0..21              */
/*  Python: struct.pack('<IHHIBBBBfH', ...) + crc16 as '<H'            */
/* ------------------------------------------------------------------ */
#define AV_JET_MAGIC   0x314A5641u
#define AV_JET_VERSION 1
#define AV_JET_LEN     24

typedef struct {
    uint32_t seq;
    uint8_t  obstacle, zone, confidence, class_id;
    float    dist_m;
    uint16_t age_ms;
} jetson_det_t;

enum { JET_OK = 0, JET_BAD_LEN, JET_BAD_MAGIC, JET_BAD_VERSION, JET_BAD_CRC, JET_BAD_VALUE };

uint16_t crc16_ccitt(const uint8_t *data, int len);
/* checks length, magic, version, CRC and every value range; fills *out on JET_OK */
int      jetson_parse(const uint8_t *buf, int len, jetson_det_t *out);
const char *jetson_err_name(int err);

#endif /* HW_PROTOCOLS_H */
