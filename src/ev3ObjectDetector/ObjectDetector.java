package ev3ObjectDetector;

import java.util.Arrays;

import ev3Odometer.Odometer;
import lejos.hardware.lcd.LCD;
import lejos.robotics.SampleProvider;

public class ObjectDetector extends Thread{
	
	SampleProvider usSensor, colorValue;
	
	Odometer odometer;
	
	public ObjectDetector(SampleProvider pUsSensor, SampleProvider pColorValue, Odometer pOdometer)
	{
		usSensor = pUsSensor;
		colorValue = pColorValue;
		odometer = pOdometer;
	}

	
	public void run()
	{
		
	}
	
	
	public void detectObject()
	{
		
	}
	
	private float getFilteredData(int sampleSize){

		float sampleData[] = new float[sampleSize];

		for(int index = 0 ; index < sampleData.length; index++)
		{
			usSensor.fetchSample(usData, 0);

			if(usData[0]*100> usSensorMaxDistance)
				usData[0] = usSensorMaxDistance;

			sampleData[index] = usData[0]*100;
			try {
				Thread.sleep(5);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}

		Arrays.sort(sampleData);
		LCD.drawString("Distance: "+sampleData[(int) Math.floor(sampleData.length/2)], 0, 4);
		return sampleData[(int) Math.floor(sampleData.length/2)];
	}

}
