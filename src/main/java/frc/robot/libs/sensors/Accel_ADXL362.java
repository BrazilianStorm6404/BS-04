package frc.robot.libs.sensors;

import edu.wpi.first.wpilibj.ADXL362;
import edu.wpi.first.wpilibj.SPI.Port;

public class Accel_ADXL362 extends ADXL362{
    /**
     * Object to use the accelerometer ADXL362
     */
    /**
     * Don't use this unless you know what you are making.
     * @param range Defines the Range of your acceleration. With less range better precision, 
     * Accel & Gyro support enum values of 2G, 4G and 8G.
     */
    protected Accel_ADXL362(Range range){
        super(range);
    }
    /**
     * Create a new accelerometer object.
     * <p> Use this when you know which port is the Accel and when there aren`t the Accel & Gyro on the SPI.
     * @param port Define which SPI port is your accelerometer.
     * @param range Defines the Range of your acceleration. With less range better precision.
     */
    public Accel_ADXL362(Port port,Range range){
        super(port,range);
    }
}