/**
 * 
 */
package de.maflo321.youduda.umfahren;

/**
 * @author youduda
 * @version 1.0
 */
public class SensorHandler implements Runnable {
	lejos.hardware.sensor.EV3UltrasonicSensor ultrasonicSensor;
	public boolean threadRunning;
	float[] ultrasonicSamples;

	/**
	 * 
	 */
	public SensorHandler(int sampleCount) {
		threadRunning = false;
		init(sampleCount);

	}

	public int init(int sampleCount) {
		ultrasonicSensor = new lejos.hardware.sensor.EV3UltrasonicSensor(
				lejos.hardware.ev3.LocalEV3.get().getPort("S4"));
		// get an instance of this sensor in measurement mode in meter unit
		// get sensor values:
		// initialize an array of floats for fetching samples.
		// Ask the SampleProvider how long the array should be
		// float[] sample = new float[distanceSensor.sampleSize()];
		// distanceSensor.fetchSample(sample, 0);

		ultrasonicSamples = new float[sampleCount]; // contain last 20 values
		return 0;
	}

	private void setUSSamples(boolean fillUp) {
		if (!fillUp) {
			for (int i = ultrasonicSamples.length - 1; i > 0; i--)
				ultrasonicSamples[i] = ultrasonicSamples[i - 1];

			float[] sample = new float[ultrasonicSensor.sampleSize()];
			ultrasonicSensor.fetchSample(sample, 0);
			ultrasonicSamples[0] = sample[0] * 100;

		} else {
			float[] sample = new float[ultrasonicSensor.sampleSize()];
			ultrasonicSensor.fetchSample(sample, 0);

			for (int i = ultrasonicSamples.length - 1; i >= 0; i--)
				ultrasonicSamples[i] = sample[0] * 100;

		}
	}

	public float[] getUltrasonicSamples() {
		return ultrasonicSamples;
	}

	@Override
	public void run() {
		threadRunning = true;

		// Fill up to have no old or unset values in samples
		setUSSamples(true);

		while (threadRunning) {
			setUSSamples(false);
			lejos.utility.Delay.msDelay(100);
		}
	}

}
