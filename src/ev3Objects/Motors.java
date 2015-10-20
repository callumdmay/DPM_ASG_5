package ev3Objects;

import ev3Odometer.Odometer;
import ev3WallFollower.UltrasonicController;
import ev3WallFollower.UltrasonicPoller;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Motors {
	
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3LargeRegulatedMotor clawMotor;
	
	private double wheelRadius;
	private double axleLength;
	
	
	public Motors (EV3LargeRegulatedMotor pLeftMotor, EV3LargeRegulatedMotor pRightMotor, EV3LargeRegulatedMotor pClawMotor, double pWheelRadius, double pAxleLength)
	{
		leftMotor 					= pLeftMotor;
		rightMotor 					= pRightMotor;
		clawMotor 					= pClawMotor;
		wheelRadius 				= pWheelRadius;
		axleLength 					= pAxleLength;
	}

	public EV3LargeRegulatedMotor getLeftMotor() {
		return leftMotor;
	}

	public EV3LargeRegulatedMotor getRightMotor() {
		return rightMotor;
	}

	public EV3LargeRegulatedMotor getClawMotor() {
		return clawMotor;
	}

	public double getWheelRadius() {
		return wheelRadius;
	}

	public double getAxleLength() {
		return axleLength;
	}
	
	

}
