package frc.robot.libs.sensors;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;

public class NavX extends AHRS{
    public NavX(){
        super();
    }
    public NavX(SerialPort.Port serial){
        super(serial);
    }

}