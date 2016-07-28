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
		Delay.msDelay(1000);
		while (true) {
			goAlong();
			Delay.msDelay(100);

		}
	}

	/**
	 * move toward the aimed distance and forward
	 */
	private void goAlong() {
		// aim must be out of the arc
		float nowDistance = getCurrentOffsetSigned();
		float nowDist = getCurrentSigned();
		float avgDist = getAverageOffsetSigned(10);
		float highDist = getHighestOffsetSigned(10);

		if (Math.abs(nowDist) > Math.abs(avgDist)) {
			// Moving away from aim distance
			if (nowDist > avgDist) {
				// Moving away from aim distance and wall
				headForPoint(-nowDistance, 200);
			} else if (nowDist < avgDist) {
				// Moving to the wall and is between aim distance and wall
				headForPoint(-nowDistance, 200);
			}
		} else if (Math.abs(nowDist) < Math.abs(highDist)) {
			// Moving to the aim distance
			// Robot is in the right direction -> do nothing
			if (nowDist > avgDist) {
				// Moving to the aim distance and is between aim distance and
				// wall
				headForPoint(0, 200);
			} else if (nowDist < avgDist) {
				// Moving to the aim distance and is not between aim distance
				// and wall
				headForPoint(0, 200);
			}
		} else if (nowDist == avgDist) {
			// Parallel to the aim
			headForPoint(-nowDistance, 200);
		}
	}

	/**
	 * This method only heads for a point, it does not go there<br>
	 * yAxis shouldn't be to low to prevent the robot from heading for the wall
	 * 
	 * @param xAxis
	 *            x-coordinate of the point
	 * @param yAxis
	 *            y-coordinate of the point, should be much higher than radius
	 */
	private void headForPoint(double direction, double xAxis, double yAxis) {
		double r = 100; // TODO: Check if aim is OUTSIDE of circle
		xAxis = r + xAxis;
		double yValue = yAxis;
		double xValue = xAxis;

		double x = 0;

		double y1 = ((2 * Math.pow(r, 2) * yValue) / Math.pow(xValue, 2)
				+ Math.sqrt(4 * Math.pow(r, 4) * Math.pow(yValue, 2) / Math.pow(xValue, 4)
						- 4 * (1 + Math.pow(yValue, 2) / Math.pow(xValue, 2))
								* (Math.pow(r, 4) / Math.pow(xValue, 2) - Math.pow(r, 2))))
				/ (2 * (1 + Math.pow(yValue, 2) / Math.pow(xValue, 2)));
		double y2 = ((2 * Math.pow(r, 2) * yValue) / Math.pow(xValue, 2)
				- Math.sqrt(4 * Math.pow(r, 4) * Math.pow(yValue, 2) / Math.pow(xValue, 4)
						- 4 * (1 + Math.pow(yValue, 2) / Math.pow(xValue, 2))
								* (Math.pow(r, 4) / Math.pow(xValue, 2) - Math.pow(r, 2))))
				/ (2 * (1 + Math.pow(yValue, 2) / Math.pow(xValue, 2)));

		double x1 = Math.sqrt(Math.pow(r, 2) - Math.pow(y1, 2));
		double x2 = Math.sqrt(Math.pow(r, 2) - Math.pow(y2, 2));

		if (xAxis < 0) { // Interval: ]-infinite; 0[ and ]r;
			if (yAxis >= r)
				x = x1;
			else
				x = -x1;
		} else if (xAxis <= r) { // Interval: [0; r]
			if (yAxis >= -r)
				x = x2;
			else if (yAxis < -r)
				x = -x2;
		} else if (xAxis > r) {

			xAxis = xAxis - r;
			if (xAxis < 0) { // Interval: ]-infinite; 0[ and ]r;
				if (yAxis >= -r)
					x = -x2;
				else
					x = x2;
			} else if (xAxis > 0) { // Interval: [0; r]
				x = x1;
			}
			r = -r; // aim is on the robot's right side
		}

		double angle = 0;
		if (r > 0) {
			angle = Math.acos(x / Math.abs(r)) * 360.0 / (2.0 * Math.PI);
			chassis.arc(-r, angle);
		} else if (r < 0) {
			angle = Math.acos(x / Math.abs(r)) * 360.0 / (2.0 * Math.PI);
			chassis.arc(-r, angle);
		} else if (r == x)
			chassis.travel(20);
	}

	private void traverseArc() {
		while (true) {
			if (getHighestOffsetUnsigned(1) < distanceOffsetArc)
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
