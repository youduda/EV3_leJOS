package de.maflo321.youduda.umfahren;

import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;
import lejos.hardware.lcd.GraphicsLCD;

/**
 * class to initialize devices and keep distance off a wall while surrounding it
 * 
 * @author youduda
 * @version 1.0
 */
public class Umfahren {
	RegulatedMotor motorLeft;
	RegulatedMotor motorRight;
	GraphicsLCD lcd;
	lejos.robotics.chassis.WheeledChassis chassis;
	SensorHandler sensors;
	float distance;
	float distanceOffsetArc;

	/**
	 * creates object to run the program
	 */
	public static void main(String[] args) {
		Umfahren proc = new Umfahren();
		if (proc.init() == 0)
			proc.run();
		else
			proc.error();
		Delay.msDelay(100);
	}

	/**
	 * Sets the sensor, motor and lcd attributes.<br>
	 * Initiate all needed devices.<br>
	 * Sets font to default.<br>
	 * Shows battery state.
	 * 
	 * @return success_error ( = 0) when finished correctly and battery is ok.
	 */
	private int init() {
		distance = 40; // distance from wall
		distanceOffsetArc = 10; // offset from distance to start traversing arc

		// only open devices once
		// Configure wheels: 5.5 cm diameter and 6 cm offset off the center
		// all other distances must be in the same unit, here cm
		lejos.robotics.chassis.Wheel wheelLeft = lejos.robotics.chassis.WheeledChassis
				.modelWheel(lejos.hardware.motor.Motor.B, 5.5).offset(-6);
		lejos.robotics.chassis.Wheel wheelRight = lejos.robotics.chassis.WheeledChassis
				.modelWheel(lejos.hardware.motor.Motor.C, 5.5).offset(6);
		chassis = new lejos.robotics.chassis.WheeledChassis(
				new lejos.robotics.chassis.Wheel[] { wheelLeft, wheelRight },
				lejos.robotics.chassis.WheeledChassis.TYPE_DIFFERENTIAL);

		// Sensors:
		sensors = new SensorHandler(100);

		lcd = lejos.hardware.BrickFinder.getDefault().getGraphicsLCD();
		lcd.setFont(lejos.hardware.lcd.Font.getDefaultFont());

		// Show battery state on the display
		float batteryState = lejos.hardware.Battery.getBatteryCurrent();
		lcd.drawString(String.valueOf(batteryState), 0, 0, 0);
		// LCD need time
		Delay.msDelay(100);
		return 0;
	}

	private void error() {
		// TODO: implement error handling
		// restart program?
	}

	private void run() {
		chassis.setAngularSpeed(10);
		chassis.setLinearSpeed(10);
		Thread thread1 = new Thread(sensors);
		thread1.start();

		while (true) {
			if (getHighestOffset(1) < distanceOffsetArc)
				traverseLine();

			else if (getHighestOffset(1) >= distanceOffsetArc)
				traverseArc();
			Delay.msDelay(50);

		}
	}

	/**
	 * handle distance difference from -30 to 30 mm
	 */
	private void traverseLine() {
		double tmp = getAverageOffset(1);
		if (tmp >= 0) {
			chassis.arc(-200, Math.acos((1000.0 - tmp) / 1000.0) * 360.0 / (2.0 * Math.PI));
		}
		if (tmp < 0) {
			chassis.arc(200, Math.acos((1000.0 + tmp) / 1000.0) * 360.0 / (2.0 * Math.PI));
		}
	}

	private void traverseArc() {
		while (true) {
			if (getHighestOffset(1) < distanceOffsetArc)
				break;
			// TODO: implement drive around edge
		}
	}

	/**
	 * reads the last values from SensorHandler and returns the average
	 * difference between the aimed distance and the distance in the last n
	 * values<br>
	 * average(distance_sensor - distance_aimed)
	 * 
	 * @param count
	 *            sets the number of values to calculate since now
	 * @return average signed offset: average(distance_sensor -
	 *         distance_aimed)<br>
	 *         positive return means to far away from the wall, negative to near
	 */
	private float getAverageOffsetSigned(int count) {
		float ret = 0;
		float[] samples = sensors.getUltrasonicSamples();
		if (count > samples.length)
			throw new IllegalArgumentException("Invalid count " + count);

		for (int i = 0; i < count; i++) {
			ret += samples[i] - distance;
		}
		return ret / count;
	}

	/**
	 * reads the last values from SensorHandler and gets the highest difference
	 * between the aimed distance and the distance in the last n values
	 * 
	 * @param count
	 *            sets the number of values to compare since now
	 * @return absolute highest offset from the aim distance unsigned
	 */
	private float getHighestOffsetUnsigned(int count) {
		return getHighestOffsetSigned(count);
	}

	/**
	 * reads the last values from SensorHandler and gets the highest difference
	 * between the aimed distance and the distance in the last n values <br>
	 * highest(distance_sensor - distance_aimed)
	 * 
	 * @param count
	 *            sets the number of values to compare since now
	 * @return absolute highest offset from the aim distance signed<br>
	 *         positive return means to far away from the wall, negative to near
	 */
	private float getHighestOffsetSigned(int count) {
		float ret = 0;
		float[] samples = sensors.getUltrasonicSamples();
		if (count > samples.length)
			throw new IllegalArgumentException("Invalid count " + count);

		for (int i = 0; i < count; i++)
			if (Math.abs(samples[i] - distance) > Math.abs(ret))
				ret = samples[i] - distance;
		return ret;
	}

	/**
	 * reads the last values from SensorHandler and gets the last one. The
	 * difference between the aimed distance and the distance in the last value
	 * is returned.<br>
	 * current(distance_sensor - distance_aimed)
	 * 
	 * @return last offset from the aim distance signed<br>
	 *         positive return means to far away from the wall, negative to near
	 */
	private float getCurrentOffsetSigned() {
		float[] samples = sensors.getUltrasonicSamples();
		if (samples.length <= 0)
			throw new NegativeArraySizeException("Invalid array size " + samples.length);
		return samples[0] - distance;
	}

	private float getCurrentSigned() {
		float[] samples = sensors.getUltrasonicSamples();
		if (samples.length <= 0)
			throw new NegativeArraySizeException("Invalid array size " + samples.length);
		return samples[0];
	}
}
