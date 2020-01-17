package frc.robot.libs.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DigitalSource;

public class Ultrasonic extends edu.wpi.first.wpilibj.Ultrasonic{
    public Ultrasonic(DigitalSource sourceA,DigitalSource sourceB){
        super((DigitalOutput)sourceA,(DigitalInput)sourceB,edu.wpi.first.wpilibj.Ultrasonic.Unit.kMillimeters);
    }
}