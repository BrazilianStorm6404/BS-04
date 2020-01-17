package frc.robot.libs.sensors;
import edu.wpi.first.wpilibj.DigitalInput;
 
public class LimitSwitch extends DigitalInput{
    public LimitSwitch(int port){
        super(port);
    }
}