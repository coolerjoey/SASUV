#include "LowPassFilter.h"
#include "mymath.h"

float LowPassFilter(const float sample, float cutoff_freq, float dt) {
	float _output;
	if (cutoff_freq <= 0.0f || dt <= 0.0f) {	
//		hal.uartD->printf("without filter \r\n");
        _output = sample;
        return _output;
	}
	float rc = 1.0f/(M_2PI*cutoff_freq);	//ʱ�䳣��t=1/(2*pi*f)
	//aΪ��RCֵ�йص�һ����������Ϊ�˲�ϵ������ֵ�����²���ֵ�ڱ����˲��������ռ��Ȩ�أ���ֵͨ��ԶС��1
	//�������ֵ��Ҫȡ�����ϴ��˲����ֵ����ǰ����ֵ�Ա���������ױȽ�С������������
	float alpha = constrain_float(dt/(dt+rc), 0.0f, 1.0f);	//dtΪ�������ڣ�������ʱ�䳣��
	_output += (sample - _output) * alpha;
	return _output;
}

