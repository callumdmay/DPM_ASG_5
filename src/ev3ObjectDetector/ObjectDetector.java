package ev3ObjectDetector;


import ev3Odometer.Odometer;
import ev3WallFollower.UltrasonicPoller;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

public class ObjectDetector{

	private SampleProvider colorValue;
	private Odometer odometer;
	private UltrasonicPoller ultraSonicPoller;
	private ObstacleAvoider obstacleAvoider;
	
	public enum OBJECT_TYPE { block, obstacle } 

	private float[] colorData;
	private final int FILTER_OUT = 5;
	private int filterControl;
	private final double obstacleDistance = 5;
	private OBJECT_TYPE currentObject;
	private boolean objectDetected;



	public ObjectDetector(UltrasonicPoller pUltraSonicPoller, SampleProvider pColorValue, float[] pColorData, Odometer pOdometer, ObstacleAvoider pObstacleAvoider)
	{
		ultraSonicPoller  = pUltraSonicPoller;
		colorValue = pColorValue;
		colorData = pColorData;
		odometer = pOdometer;
		obstacleAvoider = pObstacleAvoider;
	}


	//This method checks for obstacles in front of the robot as it is moving forward
	public boolean detectedObject()
	{

		// rudimentary filter - checks 5 times to ensure obstacle is really ahead of robot
		if( ultraSonicPoller.getDistance() < obstacleDistance)
		{
			filterControl ++;
		}

		//We must get 5 readings of less than 25 before we initiate obstacle avoidance
		if(filterControl < FILTER_OUT)
		{
			objectDetected = false;
			return false;
		}

		filterControl = 0;
		objectDetected = true;
		
		return true;
	}

	public void processObject(double pX, double pY)
	{
		if(objectIsBlock())
		{
			Sound.beep();
			currentObject = OBJECT_TYPE.block;
		}
		else
		{
			Sound.beep();
			Sound.beep();
			currentObject = OBJECT_TYPE.obstacle;
			obstacleAvoider.avoidObstacle(pX, pY);
		}
	}

	
	private boolean objectIsBlock()
	{
		return true;
	}
	
	public boolean isObjectDetected()
	{
		return objectDetected;
	}

	public OBJECT_TYPE getCurrentObject() {
		return currentObject;
	}


	


}
