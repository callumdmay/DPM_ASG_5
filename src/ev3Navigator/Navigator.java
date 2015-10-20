package ev3Navigator;

import java.util.ArrayList;

import ev3ObjectDetector.ObjectDetector;
import ev3ObjectDetector.ObstacleAvoider;
import ev3Objects.FoundBlockException;
import ev3Objects.Motors;
import ev3Odometer.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigator extends Thread{

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3LargeRegulatedMotor clawMotor;

	private double wheelRadius;
	private double axleLength;
	private static final double  tileLength = 30.48;

	private final double locationError = 1;
	private final double navigatingAngleError = 1;

	private final int FORWARD_SPEED = 200;
	private final int ROTATE_SPEED = 100;
	private final int SMALL_CORRECTION_SPEED =40;
	private final int SMALL_ROTATION_SPEED = 25;

	private final double[][] arenaBoundary = { {-0.6,-0.6}, {2.6,2.6} };

	private ArrayList<Coordinate> arenaBoundaryCoordinates;
	private Odometer odometer;
	private ObjectDetector objectDetector;

	private ArrayList<Coordinate> objectCoordinates = new ArrayList<Coordinate>();

	public static int coordinateCount = 0;
	private static ArrayList<Coordinate> coordinates;


	public Navigator(Odometer pOdometer, Motors pMotors, ObjectDetector pObjectDetector)
	{
		odometer 					= pOdometer;
		leftMotor 					= pMotors.getLeftMotor();
		rightMotor 					= pMotors.getRightMotor();
		clawMotor 					= pMotors.getClawMotor();
		wheelRadius 				= pMotors.getWheelRadius();
		axleLength 					= pMotors.getAxleLength();
		objectDetector 				= pObjectDetector;

		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] { leftMotor, rightMotor }) {
			motor.stop();
			motor.setAcceleration(2000);

		}

		arenaBoundaryCoordinates = createCoordinatesList(arenaBoundary);
	}

	@Override
	public void run()
	{
		Sound.beepSequenceUp();

		//Move to the first coordinate and face to the next coordinate
		double currentX = coordinates.get(0).getX();
		double currentY = coordinates.get(0).getY();
		double pX = coordinates.get(1).getX();
		double pY = coordinates.get(1).getY();

		travelTo(currentX, currentY);

		turnTo(	NavigatorUtility.calculateNewAngle(pX - currentX, pY - currentY), false);
		coordinates.remove(0);

		//Initiate search pattern, defined by input coordinates to navigator
		try{

			for(Coordinate coordinate: coordinates)
			{
				travelTo(coordinate.getX(), coordinate.getY());
			}

		}
		catch(FoundBlockException e)
		{
			//capture the block and travel to the last coordinate point, which is always the end zone
			captureBlock();
			travelTo(coordinates.get(coordinates.size()).getX(), coordinates.get(coordinates.size()).getY());
		}

	}

	//This method takes a new x and y location, and moves to it while avoiding obstacles
	public void travelTo(double pX, double pY)
	{
		//While the robot is not at the objective coordinates, keep moving towards it 
		while(Math.abs(pX- odometer.getX()) > locationError || Math.abs(pY - odometer.getY()) > locationError)
		{
			if(objectDetector.detectedObject())
			{


				double objectDistance = objectDetector.getObjectDistance();
				double objectX = objectDistance * Math.cos(odometer.getTheta());
				double objectY =  objectDistance * Math.sin(odometer.getTheta());

				if(objectX >= arenaBoundaryCoordinates.get(0).getX() && objectY >=arenaBoundaryCoordinates.get(0).getY())
				{
					Sound.beep();
					stopMotors();
					investigateObject(objectX,objectY);

					if(objectDetector.getCurrentObject() == ObjectDetector.OBJECT_TYPE.block)
						throw new FoundBlockException();
					
					if(objectDetector.getCurrentObject() == ObjectDetector.OBJECT_TYPE.obstacle)
						if(odometer.getX() <= tileLength * 1.6 && odometer.getY() <= tileLength * 1.6)
						{
							objectDetector.obstacleAvoider.squareAvoid(10, ObstacleAvoider.DIRECTION.left);
						}
						else
						{
							objectDetector.obstacleAvoider.squareAvoid(10, ObstacleAvoider.DIRECTION.right);
						}
				}
			}

			moveToCoordinates(pX, pY);
		}

	}

	//Turns to the absolute value theta
	public void turnTo(double pTheta, boolean useSmallRotationSpeed)
	{

		pTheta = pTheta % Math.toRadians(360);

		double deltaTheta = pTheta - odometer.getTheta();

		double rotationAngle = 0;

		if( Math.abs(deltaTheta) <= Math.PI)
			rotationAngle = deltaTheta;

		if(deltaTheta < -Math.PI)
			rotationAngle = deltaTheta + 2*Math.PI;

		if(deltaTheta > Math.PI)
			rotationAngle = deltaTheta - 2*Math.PI;

		//Basic proportional control on turning speed when
		//making a small angle correction
		if(Math.abs(deltaTheta)<= Math.toRadians(10) || useSmallRotationSpeed)
		{
			leftMotor.setSpeed(SMALL_ROTATION_SPEED);
			rightMotor.setSpeed(SMALL_ROTATION_SPEED);
		}
		else
		{
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
		}

		leftMotor.rotate(-NavigatorUtility.convertAngle(wheelRadius, axleLength, rotationAngle * 180/Math.PI), true);
		rightMotor.rotate(NavigatorUtility.convertAngle(wheelRadius, axleLength, rotationAngle * 180/Math.PI), false);
	}




	/*
	 * This method simply navigates to the given coordinates
	 */
	private void moveToCoordinates(double pX, double pY)
	{
		double currentX = odometer.getX();
		double currentY = odometer.getY();

		double newAngle = NavigatorUtility.calculateNewAngle(pX - currentX, pY - currentY);


		if(Math.abs(  Math.toDegrees(NavigatorUtility.calculateShortestTurningAngle(newAngle, odometer.getTheta())))  > navigatingAngleError)
			turnTo(newAngle, false);
		else
		{
			//Basic proportional control, when the robot gets close to 
			//required coordinates, slow down 
			if(Math.abs(pX - currentX) <= 3 && Math.abs(pY - currentY ) <= 3)
			{
				leftMotor.setSpeed(SMALL_CORRECTION_SPEED);
				rightMotor.setSpeed(SMALL_CORRECTION_SPEED);
			}
			else
			{
				leftMotor.setSpeed(FORWARD_SPEED);
				rightMotor.setSpeed(FORWARD_SPEED);
			}
			leftMotor.forward();
			rightMotor.forward();
		}
	}



	private void investigateObject(double pX, double pY)
	{

		while(objectDetector.getObjectDistance() >=4)
			moveToCoordinates(pX, pY);

		objectDetector.processObject();
	}

	private void captureBlock()
	{
		clawMotor.rotate(210);
		clawMotor.flt();
	}


	private static ArrayList<Coordinate> createCoordinatesList( double coordinates[][])
	{
		ArrayList<Coordinate> coordinatesQueue = new ArrayList<Coordinate>();

		for (int x = 0 ; x < coordinates.length; x++)
			coordinatesQueue.add(new Coordinate(coordinates[x][0]*tileLength,coordinates[x][1]*tileLength));

		return coordinatesQueue;
	}



	//Sets the global coordinates for the navigator
	public void setCoordinates(double pCoordinates[][])
	{
		coordinates = createCoordinatesList(pCoordinates);
	}

	public void stopMotors()
	{
		leftMotor.stop();
		rightMotor.stop();
	}

	public void rotateClockWise(int speed)
	{
		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);

		leftMotor.forward();
		rightMotor.backward();
	}

	public void rotateCounterClockWise(int speed)
	{
		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);

		leftMotor.backward();
		rightMotor.forward();
	}


}

