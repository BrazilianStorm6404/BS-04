package frc.robot.libs.sensors;

import edu.wpi.first.wpilibj.PIDSourceType;

public class AccelAndGyro extends Accel_ADXL362{
    /**
     * Gyroscope object
     */
    protected Gyro_ADXRS450 gyro;
    /**
     * Create an object of the Accel&Gyro
     * @param range Defines the Range of your acceleration. With less range better precision, 
     * Accel & Gyro support enum values of 2G, 4G and 8G.
     */
    public AccelAndGyro(Range range){
        super(range);
        gyro = new Gyro_ADXRS450();
    }
    /**
     * Define the actual position to zero.
     */
    public void resetGyro(){
        gyro.reset();
    }
    /**
     * Get the angle from gyroscope
     * @return Return the actual angle from the gyroscope
     */
    public double getAngle(){
        return gyro.getAngle();
    }
    /**
     * Get the type of PID are the Gyro;
     * @return Return the PID type from gyro;
     */
    public PIDSourceType getPIDGyroType(){
        return gyro.getPIDSourceType();
    }
}