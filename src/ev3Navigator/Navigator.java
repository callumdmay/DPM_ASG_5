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
	private static final double  tileLength = 30.9;

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

		turnTo(0, false);
		travelTo(0,0);
		//Initiate search pattern, defined by input coordinates to navigator
		try{

			for(Coordinate coordinate: coordinates)
			{
				travelTo(coordinate.getX(), coordinate.getY());
				scanForObjects();
			}

		}
		catch(FoundBlockException e)
		{
			//capture the block and travel to the last coordinate point, which is always the end zone
			captureBlock();
			double pX = coordinates.get(coordinates.size()-1).getX();
			double pY = coordinates.get(coordinates.size()-1).getY();
			while(Math.abs(pX- odometer.getX()) > locationError || Math.abs(pY - odometer.getY()) > locationError)
			{
				moveToCoordinates(pX, pY);
			}
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

				Sound.beep();
				stopMotors();
				investigateObject();

				//Determine result of investigation
				if(objectDetector.getCurrentObject() == ObjectDetector.OBJECT_TYPE.block)
					throw new FoundBlockException();

				if(objectDetector.getCurrentObject() == ObjectDetector.OBJECT_TYPE.obstacle)
				{
					objectDetector.obstacleAvoider.squareAvoid(10, ObstacleAvoider.DIRECTION.right);
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

	private void scanForObjects()
	{
		stopMotors();
		double endingTheta = odometer.getTheta() - Math.toRadians(5);
		//apply wrap around to maintain angles in [0,360]
		if(endingTheta < 0)
			endingTheta += Math.toRadians(360);
		
		//while the robot has not done a full revolution, scan for objects
		while(Math.abs(odometer.getTheta() - endingTheta )>= Math.toRadians(2))
		{	
			rotateCounterClockWise(50);
			if(objectDetector.detectedObject())
			{
				stopMotors();

				Sound.beep();
				double currentXLoc = odometer.getX();
				double currentYLoc = odometer.getY();

				//capture the object angle
				double objectAngle = odometer.getTheta();
				//turn a little more towards the object
				turnTo(objectAngle+ Math.toRadians(17), false);

				investigateObject();

				if(objectDetector.getCurrentObject() == ObjectDetector.OBJECT_TYPE.block)
					throw new FoundBlockException();

				if(objectDetector.getCurrentObject() == ObjectDetector.OBJECT_TYPE.obstacle)
				{
					leftMotor.rotate(-450, true);
					rightMotor.rotate(-450, false);
					moveToCoordinates(currentXLoc, currentYLoc);
					turnTo(objectAngle+Math.toRadians(90),false);
				}

			}
		}

	}

	private void investigateObject()
	{
		while(objectDetector.getObjectDistance() >=6 )
		{
			if(objectDetector.getObjectDistance()> objectDetector.getDefaultObstacleDistance())
				break;
			driveStraight(30);
		}

		objectDetector.processObject();
	}

	private void captureBlock()
	{
		clawMotor.setSpeed(50);
		clawMotor.rotate(-80);
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
		leftMotor.stop(true);
		rightMotor.stop(false);
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

	public void driveStraight(int speed)
	{
		leftMotor.setSpeed(speed);
		rightMotor.setSpeed(speed);

		leftMotor.forward();
		rightMotor.forward();
	}

}

