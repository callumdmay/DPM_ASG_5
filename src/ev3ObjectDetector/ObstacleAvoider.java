package ev3ObjectDetector;

import ev3Navigator.NavigatorUtility;
import ev3Objects.Motors;
import ev3Odometer.Odometer;
import ev3WallFollower.UltrasonicController;
import ev3WallFollower.UltrasonicPoller;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class ObstacleAvoider {

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	public enum DIRECTION {left , right};

	private Odometer odometer;
	private UltrasonicPoller ultraSonicPoller;
	private UltrasonicController wallFollowerController;

	private double wheelRadius;
	private double axleLength;

	private final int neckMotor_OFFSET = 60;
	private final double wallFollowingAngleError = 4 ;


	public ObstacleAvoider(Odometer pOdometer, UltrasonicPoller pUltraSonicPoller, UltrasonicController pwallFollowerController, Motors pMotors)
	{
		ultraSonicPoller 			= pUltraSonicPoller;
		wallFollowerController 		= pwallFollowerController;
		odometer 					= pOdometer;
		leftMotor 					= pMotors.getRightMotor();
		rightMotor 					= pMotors.getRightMotor();
		wheelRadius 				= pMotors.getWheelRadius();
		axleLength 					= pMotors.getAxleLength();
	}


	/*
	 * This method basically runs the p-type wall-following algorithm
	 *until the robot is facing back at towards coordinates. It then tries to move towards them again
	 */

	public void avoidObstacle(double pX, double pY) {


		rightMotor.stop();
		leftMotor.stop();

		leftMotor.rotate(NavigatorUtility.convertAngle(wheelRadius, axleLength, 90), true);
		rightMotor.rotate(-NavigatorUtility.convertAngle(wheelRadius, axleLength, 90), false);

		double currentX;
		double currentY;

		do{
			currentX = odometer.getX();
			currentY = odometer.getY();
			wallFollowerController.processUSData((int) ultraSonicPoller.getDistance());
		} while(Math.abs(NavigatorUtility.calculateAngleError(pX - currentX, pY - currentY, odometer.getTheta())*180/Math.PI) > wallFollowingAngleError);

	}

	public void squareAvoid(double length, DIRECTION direction )
	{
		leftMotor.rotate(NavigatorUtility.convertDistance(wheelRadius, -axleLength/2));
		rightMotor.rotate(NavigatorUtility.convertDistance(wheelRadius, -axleLength/2), true);

		double angle;
		if(direction.equals(ObstacleAvoider.DIRECTION.left))
			angle = 90;
		else if(direction.equals(ObstacleAvoider.DIRECTION.right))
			angle = -90;
		else
		{
			throw new IllegalArgumentException();
		}
		
		leftMotor.rotate(NavigatorUtility.convertAngle(wheelRadius, axleLength, Math.toRadians(-angle)));
		rightMotor.rotate(NavigatorUtility.convertAngle(wheelRadius, axleLength, Math.toRadians(angle)), true);
		leftMotor.rotate(NavigatorUtility.convertDistance(wheelRadius, length));
		rightMotor.rotate(NavigatorUtility.convertDistance(wheelRadius, length), true);
		leftMotor.rotate(NavigatorUtility.convertAngle(wheelRadius, axleLength, Math.toRadians(angle)));
		leftMotor.rotate(NavigatorUtility.convertDistance(wheelRadius, length+ axleLength/2));
		rightMotor.rotate(NavigatorUtility.convertDistance(wheelRadius, length + axleLength/2), true);
		leftMotor.rotate(NavigatorUtility.convertAngle(wheelRadius, axleLength, Math.toRadians(angle)));
		rightMotor.rotate(NavigatorUtility.convertAngle(wheelRadius, axleLength, Math.toRadians(-angle)), true);
		leftMotor.rotate(NavigatorUtility.convertDistance(wheelRadius, length));
		rightMotor.rotate(NavigatorUtility.convertDistance(wheelRadius, length), true);
		leftMotor.rotate(NavigatorUtility.convertAngle(wheelRadius, axleLength, Math.toRadians(-angle)));
		rightMotor.rotate(NavigatorUtility.convertAngle(wheelRadius, axleLength, Math.toRadians(angle)), true);

	}

}
