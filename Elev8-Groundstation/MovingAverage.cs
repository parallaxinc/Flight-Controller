using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Elev8
{
	public class MovingAverage
	{
		float sampleSum = 0;
		int sampleIndex = 0;
		float[] samples = null;

		public MovingAverage( int NumSamples )
		{
			samples = new float[NumSamples];
		}

		public void AddSample( float s )
		{
			sampleSum -= samples[sampleIndex];
			samples[sampleIndex] = s;
			sampleSum += s;

			sampleIndex = (sampleIndex + 1) % samples.Length;
		}

		public float Value
		{
			get {
				return sampleSum / (float)samples.Length;
			}
		}
	}
}
