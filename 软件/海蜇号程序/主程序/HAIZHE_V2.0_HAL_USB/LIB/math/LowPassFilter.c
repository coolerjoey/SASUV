#include "LowPassFilter.h"
#include "mymath.h"

float LowPassFilter(const float sample, float cutoff_freq, float dt) {
	float _output;
	if (cutoff_freq <= 0.0f || dt <= 0.0f) {	
//		hal.uartD->printf("without filter \r\n");
        _output = sample;
        return _output;
	}
	float rc = 1.0f/(M_2PI*cutoff_freq);	//时间常数t=1/(2*pi*f)
	//a为与RC值有关的一个参数，称为滤波系数，其值决定新采样值在本次滤波结果中所占的权重，其值通常远小于1
	//本次输出值主要取决于上次滤波输出值，当前采样值对本次输出贡献比较小，起到修正作用
	float alpha = constrain_float(dt/(dt+rc), 0.0f, 1.0f);	//dt为采样周期，不等于时间常数
	_output += (sample - _output) * alpha;
	return _output;
}

