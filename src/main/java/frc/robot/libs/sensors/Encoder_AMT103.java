package frc.robot.libs.sensors;

import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.Encoder;

public class Encoder_AMT103 extends Encoder{
    /**
     * Create a Encoder object.
     * <p> If the encoder are with the Index channel, then use annoter constructor.
     * @param A The first channel.
     * @param B The second channel.
     * @param sense direction of the encoder, defalt is true, reverse is false. 
     * @param dpp Define the distance per pulse measured by the encoder
     */
    public Encoder_AMT103(DigitalSource A,DigitalSource B, boolean sense, double dpp){
        super(A,B,sense,EncodingType.k4X);
    }
    /**
     * Create a Encoder object.
     * <p> If the encoder are with the Index channel, then use annoter constructor.
     * <p> Use this constructor make sure to set the distance per pulse to take corectly.
     * @param A The first channel.
     * @param B The second channel.
     * @param sense direction of the encoder, defalt is true, reverse is false. 
     */
    public Encoder_AMT103(DigitalSource A,DigitalSource B, boolean sense){
        super(A,B,sense,EncodingType.k4X);
    }
}