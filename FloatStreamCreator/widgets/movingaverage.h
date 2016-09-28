#ifndef MOVINGAVERAGE_H
#define MOVINGAVERAGE_H

class MovingAverage
{
public:
	MovingAverage();
	~MovingAverage();

	void Init(int NumSamples);

	void AddSample( float s );
	float Value(void) const { return sampleSum / (float)sampleCount; }

private:
	float sampleSum;

	int sampleIndex, sampleCount;
	float * samples;
};

#endif
