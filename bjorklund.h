#ifndef BJORKLUND_H
#define BJORKLUND_H

#include <Arduino.h>
#include <cmath>


#define CLIP(x) if (x < -32767) x = -32767; if (x > 32767) x = 32767;
#define CONSTRAIN(x, lb, ub) do { if (x < (lb)) x = lb; else if (x > (ub)) x = ub; }


struct {
    uint32_t pattern;
    uint8_t length;
} SEQ_PATTERN;

//Adapted from https://github.com/mxmxmx/temps_utile-/blob/master/soft/t_u_REV/APP_CLK.ino#L1374

uint32_t euclidean(uint32_t step, uint32_t beat, uint32_t offset)
{
// the three parameters:
uint32_t _n, _k, _offset;
uint32_t rhythm_bit = 0;
uint16_t _out = 0;
_n = step;
_k = beat;
_offset = step-offset;
    for (int clk_cnt_ = 0;clk_cnt_ < step;clk_cnt_++) {
        clk_cnt_ = clk_cnt_ >= (uint8_t)_n ? 0x0 : clk_cnt_;
        _out = ((clk_cnt_ + _offset) * _k) % _n;
        _out = (_out < _k) ? 1 : 0;
        if (_out)
            rhythm_bit = rhythm_bit | (1 << clk_cnt_);
    }
    return rhythm_bit;
}

//Adapted from https://github.com/Zirafkend/Bjorklund/blob/master/Bjorklund.h

uint16_t bjorklund(int step, int pulse) {
    int pauses = step - pulse;
    int remainder = pauses % pulse;
    int per_pulse = (int) floor(pauses / pulse);
    int noskip = (remainder == 0) ? 0 : (int) floor(pulse / remainder);
    int skipXTime = (noskip == 0) ? 0 : (int) floor((pulse - remainder)/noskip);
    uint16_t rhythm_bit;
    int count_bit = 0;

    int count = 0;
    int skipper = 0;
    for (int i = 1; i <= step; i++) {
        if (count == 0) {
            rhythm_bit = rhythm_bit | (1 << count_bit);
            count_bit++;
            count = per_pulse;
            if (remainder > 0 && skipper == 0) {
                count++;
                remainder--;
                skipper = (skipXTime > 0) ? noskip : 0;
                skipXTime--;
            } else {
                skipper--;
            }
        } else {
            count_bit++;
            count--;
        }
    }
    return rhythm_bit;
}

#endif // BJORKLUND_H
