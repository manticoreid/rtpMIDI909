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



uint32_t euclid_mask_full = 0b11111111111111111111111111111111;

uint16_t euclid_index[32] = {
    0,2,5,9,14,20,27,35,44,54,65,77,90,104,
    119,135,152,170,189,209,230,252,275,299,
    324,350,377,405,434,464,495,527
};

uint32_t euclid_pattern[560] = {0,1,0,1,3,0,1,3,7,0,1,5,7,15,0,1,5,21,15,31,0
,1,9,21,27,31,63,0,1,9,21,85,91,63,127,0,1,17,73,85,109,119
,127,255,0,1,17,73,85,341,219,375,255,511,0,1,33,73,165,341,693,731,495
,511,1023,0,1,33,273,585,341,1365,877,955,1519,1023,2047,0,1,65,273,585,1189,1365
,1717,1755,1911,2015,2047,4095,0,1,65,273,585,1321,1365,5461,5549,5851,6007,6111,4095,8191,0
,1,129,1057,1161,4681,2709,5461,10965,7021,11739,7927,8127,8191,16383,0,1,129,1057,4369,4681,5285
,5461,21845,22197,14043,15291,15855,24511,16383,32767,0,1,257,1057,4369,4681,18761,19093,21845,27349,28013,46811
,30583,48623,32639,32767,65535,0,1,257,4161,4369,17545,37449,38053,21845,87381,54965,56173,60891,96119,64495,98175
,65535,131071,0,1,513,4161,8721,18577,37449,42281,43605,87381,174933,177581,112347,187835,192375,128991,130815,131071,262143
,0,1,513,4161,33825,69905,37449,84297,86693,87381,349525,350901,355693,374491,244667,253687,391135,392959,262143,524287,0
,1,1025,16513,33825,69905,74825,299593,169125,305749,349525,437077,710325,449389,749275,489335,507375,520159,523775,524287,1048575,0
,1,1025,16513,33825,69905,148617,299593,600361,346773,349525,1398101,1403605,896429,898779,1502683,1537911,1555951,1040319,1572351,1048575,2097151
,0,1,2049,16513,67617,270865,559377,299593,1198665,1217701,698709,1398101,2796885,1758901,1796973,2995931,1956795,2027383,3112431,3137471,2096127
,2097151,4194303,0,1,2049,65793,266305,279073,1118481,1123401,2396745,1353001,2443925,1398101,5592405,3500757,5682605,3595117,3895003,3914683,6156023
,4127727,4177855,6290431,4194303,8388607,0,1,4097,65793,266305,1082401,1118481,2245769,2396745,4802889,4871333,4893013,5592405,6991189,7034549,7171437
,7190235,7794139,7829367,8118007,8255455,8355711,8386559,8388607,16777215,0,1,4097,65793,266305,1082401,1118481,2377873,2396745,5392969,5412005,5581461
,5592405,22369621,22391509,22730421,22768493,23967451,24042939,24606583,16236015,25032671,25132927,25163775,16777215,33554431,0,1,8193,262657,532545,1082401,2236689
,4753681,4792905,19173961,10822953,11096741,11183445,22369621,44741973,44915381,45462957,28760941,47937243,48094139,49215351,49790447,50067423,33488767,33550335,33554431,67108863,0
,1,8193,262657,2113665,4261921,4465169,17895697,9577609,19173961,21580105,38966437,22325845,22369621,89478485,89566037,56284853,91057517,57521883,95907291,62634939,98496375
,66026991,66580447,66977535,100659199,67108863,134217727,0,1,16385,262657,2113665,4327489,17318945,17895697,19022985,19173961,76698185,43296041,44386965,78292309,89478485
,111850837,179661525,181843373,115039085,191739611,192343515,125269879,129883895,199195631,133160895,201195263,134209535,134217727,268435455,0,1,16385,1049601,2113665,17043521,34636833
,17895697,71600273,71901769,153391689,153692457,88757413,156543573,89478485,357913941,223783765,359356085,229485997,230087533,249263835,250469819,393705335,259776247,264174575,401596351,268173055,402644991
,268435455,536870911,0,1,32769,1049601,4227201,17043521,34636833,69345553,143167761,76620873,153391689,306858313,173184165,312822421,178951509,357913941,715838805,448096981,727373493
,460025197,460175067,767258331,501070779,518977399,519552495,528349151,803200959,536346111,536854527,536870911,1073741823,0,1,32769,1049601,16843009,17043521,34636833,138682897,286331153
,287458441,153391689,345133641,614802729,623530661,357739093,357913941,1431655765,1432005461,900422325,917878189,1457216365,1533916891,997649883,1002159035,1038020471,1593294319,1602090975,1069531071,1610087935,1610596351
,1073741823,2147483647,0,1,65537,4196353,16843009,67641409,69272609,142885409,286331153,304367761,306778697,1227133513,1229539657,1246925989,1251297941,1252693333,1431655765,1789580629,1792371413
,1801115317,1835887981,1840700269,3067852507,3077496251,2004318071,3151884023,3186605551,2130442207,2139062143,2146434559,2147450879,2147483647,4294967295};

uint32_t euclidean_arr(uint8_t step, uint8_t beat, uint8_t offset)
{
    uint32_t pattern =  euclid_pattern[euclid_index[step]+beat];
    uint8_t offset_clamp = offset % step;
    uint32_t pattern_off = ((pattern<<(offset_clamp))&(euclid_mask_full >> (31-(step-1))))|(pattern>> (step-((offset_clamp))));
//	printf("off: %u\n", ((offset%step)) );
//	printf("mask: %u\n", ((euclid_mask_full >> (31-(step-1)))) );
//	printf("%i %i\n", pattern, pattern_off);
    return pattern_off;
}

uint32_t euclidean_arr(uint8_t step, uint8_t beat)
{
    uint32_t pattern =  euclid_pattern[euclid_index[step]+beat];
    return pattern;
}



//Adapted from https://github.com/mxmxmx/temps_utile-/blob/master/soft/t_u_REV/APP_CLK.ino#L1374

uint32_t euclidean(uint8_t step, uint8_t beat, uint8_t offset)
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

class EuclideanModule
{
  public:
    uint32_t pattern;
    uint8_t stepsize;
    uint8_t beatnum;
    uint8_t offsetnum;
    bool rollback;
    uint8_t curstep;

    EuclideanModule() {
        curstep = 0;
        pattern = 0;
        stepsize = 1;
        beatnum = 0;
        offsetnum = 0;
        rollback = false;
    }

    void setPattern(uint8_t step, uint8_t beat, uint8_t offset){
        stepsize = step;
        beatnum = beat;
        offsetnum = offset;
        pattern = euclidean_arr(step, beat, offset);
    }

    void setPattern(uint8_t step, uint8_t beat){
        setPattern(step,beat,0);
    }

    void setBeat(uint8_t beat) {
        if (beat>stepsize) stepsize = beat;
        setPattern(stepsize, beat, offsetnum);
    }

    void setStep(uint8_t step) {
        if (beatnum>step) beatnum = step;
        setPattern(step, beatnum, offsetnum);
    }

    void setOffset(uint8_t offset) {
        setPattern(stepsize, beatnum, offset);
    }

    uint32_t nextPattern()
    {
        uint32_t pat = getPattern(curstep);
        curstep++;
        if (curstep> (stepsize-1) ) curstep = 0;
        return pat;
    }

    uint32_t getPattern(uint8_t step)
    {
        return (pattern & (1<<step));
    }

    uint32_t getPatternTick(uint32_t step)
    {
        if (rollback){
           curstep = (curstep+1)%stepsize;
           rollback = false;
        }else{
           curstep = step%stepsize;
        }
        return (pattern & (1<<curstep));
    }

    void setRollback()
    {
        rollback = true;
    }


};



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
