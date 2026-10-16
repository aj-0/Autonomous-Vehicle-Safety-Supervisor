/*
 * hw_protocols.c - TF-Luna frame parser, encoder speed estimator, Jetson packet check.
 * Plain C, no hardware access: the same code runs on the Pi and in the PC unit tests.
 */
#include <math.h>
#include <string.h>
#include "hw_protocols.h"

/* ---------------- TF-Luna ---------------- */

void tfluna_init(tfluna_parser_t *p)
{
    memset(p, 0, sizeof *p);
}

int tfluna_feed(tfluna_parser_t *p, uint8_t b, tfluna_frame_t *out)
{
    /* resynchronise on the two header bytes */
    if ((p->n == 0 || p->n == 1) && b != 0x59) {
        p->n = 0;
        return 0;
    }
    p->buf[p->n++] = b;
    if (p->n < 9)
        return 0;
    p->n = 0;

    uint8_t sum = 0;
    for (int i = 0; i < 8; i++)
        sum = (uint8_t)(sum + p->buf[i]);
    if (sum != p->buf[8]) {
        p->bad_checksum++;
        return 0;
    }
    p->frames++;
    out->dist_m = (float)(p->buf[2] | (p->buf[3] << 8)) / 100.0f;
    out->amp = (uint16_t)(p->buf[4] | (p->buf[5] << 8));
    out->temp_c = (float)(p->buf[6] | (p->buf[7] << 8)) / 8.0f - 256.0f;
    return 1;
}

/* ---------------- wheel encoder ---------------- */

void enc_init(enc_est_t *e)
{
    memset(e, 0, sizeof *e);
}

void enc_pulse(enc_est_t *e, uint64_t t_us, uint32_t min_gap_us)
{
    if (e->count) {
        uint64_t last = e->t[(e->head + ENC_HIST - 1) % ENC_HIST];
        if (t_us < last + min_gap_us)
            return;                                   /* contact bounce / noise */
    }
    e->t[e->head] = t_us;
    e->head = (e->head + 1) % ENC_HIST;
    if (e->count < ENC_HIST)
        e->count++;
    e->total++;
}

float enc_speed(enc_est_t *e, uint64_t now_us, float m_per_pulse, uint32_t window_us,
                float max_accel)
{
    float raw = 0.0f;
    if (e->count) {
        uint64_t newest = e->t[(e->head + ENC_HIST - 1) % ENC_HIST];
        uint64_t oldest = newest;
        int n = 1;
        /* pulses inside the window, newest first */
        for (int i = 2; i <= e->count; i++) {
            uint64_t t = e->t[(e->head + ENC_HIST - i) % ENC_HIST];
            if (now_us - t > window_us)
                break;
            oldest = t;
            n++;
        }
        if (now_us - newest <= window_us && n >= 2 && newest > oldest)
            raw = (float)(n - 1) * m_per_pulse / ((float)(newest - oldest) * 1e-6f);
        /* no new pulse for longer than one period: the wheel is slowing down */
        if (now_us > newest) {
            float bound = m_per_pulse / ((float)(now_us - newest) * 1e-6f);
            if (raw > bound)
                raw = bound;
        }
        if (now_us - newest > window_us)
            raw = 0.0f;
    }

    float dt = e->last_update_us && now_us > e->last_update_us
                   ? (float)(now_us - e->last_update_us) * 1e-6f : 0.01f;
    e->last_update_us = now_us;
    float step = max_accel * dt;
    float d = raw - e->v;
    if (d > step)
        d = step;
    if (d < -step)
        d = -step;
    e->v += d;
    if (e->v < 1e-4f)
        e->v = 0.0f;
    return e->v;
}

/* ---------------- Jetson packet ---------------- */

uint16_t crc16_ccitt(const uint8_t *data, int len)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= (uint16_t)(data[i] << 8);
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

static uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

int jetson_parse(const uint8_t *b, int len, jetson_det_t *out)
{
    if (len != AV_JET_LEN || le16(b + 6) != AV_JET_LEN)
        return JET_BAD_LEN;
    if (le32(b) != AV_JET_MAGIC)
        return JET_BAD_MAGIC;
    if (le16(b + 4) != AV_JET_VERSION)
        return JET_BAD_VERSION;
    if (crc16_ccitt(b, 22) != le16(b + 22))
        return JET_BAD_CRC;

    jetson_det_t d;
    uint32_t raw = le32(b + 16);
    memcpy(&d.dist_m, &raw, sizeof d.dist_m);
    d.seq = le32(b + 8);
    d.obstacle = b[12];
    d.zone = b[13];
    d.confidence = b[14];
    d.class_id = b[15];
    d.age_ms = le16(b + 20);
    if (d.obstacle > 1 || d.zone > 3 || d.confidence > 100 || d.age_ms > 1000 ||
        !isfinite(d.dist_m) || d.dist_m > 100.0f || (d.dist_m < 0.0f && d.dist_m != -1.0f))
        return JET_BAD_VALUE;
    *out = d;
    return JET_OK;
}

const char *jetson_err_name(int err)
{
    switch (err) {
    case JET_OK:          return "ok";
    case JET_BAD_LEN:     return "wrong length";
    case JET_BAD_MAGIC:   return "wrong magic";
    case JET_BAD_VERSION: return "wrong version";
    case JET_BAD_CRC:     return "CRC error";
    default:              return "value out of range";
    }
}
