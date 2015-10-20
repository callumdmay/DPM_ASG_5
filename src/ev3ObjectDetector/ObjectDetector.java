package ev3ObjectDetector;


import java.util.Arrays;

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
	private final double obstacleDistance = 20;
	private OBJECT_TYPE currentObject;
	private boolean objectDetected;

	private Object lock = new Object();

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
			filterControl++;
		}

		if(filterControl < FILTER_OUT)
		{
			synchronized(lock)
			{
				objectDetected = false;
				currentObject = null;
			}
			return false;
		}

		filterControl = 0;
		synchronized(lock)
		{
			objectDetected = true;
		}
		return true;
	}



	public void processObject()
	{
		if(ultraSonicPoller.getDistance() <=6 && currentObject == null)
		{
			colorValue.fetchSample(colorData, 0);
			if(colorData[0]== 2){
				Sound.beep();
				currentObject = OBJECT_TYPE.block;
			}
			else
			{
				Sound.beep();
				Sound.beep();
				currentObject = OBJECT_TYPE.obstacle;
			}
		}
	}

	public boolean isObjectDetected()
	{
		boolean returnedValue;
		synchronized(lock)
		{
			returnedValue = objectDetected;
		}
		return returnedValue;
	}

	public OBJECT_TYPE getCurrentObject() {
		OBJECT_TYPE returnedValue;
		synchronized(lock)
		{
			returnedValue = currentObject;
		}	
		return returnedValue;
	}

	public double getObjectDistance(){

		return ultraSonicPoller.getDistance();
	}




}
