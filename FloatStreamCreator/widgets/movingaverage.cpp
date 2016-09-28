#include "movingaverage.h"
#include <string.h>

MovingAverage::MovingAverage()
{
	sampleIndex = 0;
	sampleCount = 0;
	samples = 0;
	sampleSum = 0;
}

MovingAverage::~MovingAverage()
{
	delete [] samples;
}

void MovingAverage::Init(int NumSamples)
{
	if( samples != 0 ) {
		delete [] samples;
		samples = 0;
	}

	sampleCount = NumSamples;
	samples = new float[ sampleCount ];
	sampleIndex = 0;
	sampleSum = 0;

	memset( samples, 0, sizeof(float) * sampleCount );
}

void MovingAverage::AddSample( float s )
{
	sampleSum -= samples[sampleIndex];
	samples[sampleIndex] = s;
	sampleSum += s;

	sampleIndex = (sampleIndex + 1) % sampleCount;
}
