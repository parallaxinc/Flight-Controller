/*
  Elev8 GroundStation

  Copyright 2015 Parallax Inc

  This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
  http://creativecommons.org/licenses/by-nc-sa/4.0/

  Written by Jason Dorie
*/

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
