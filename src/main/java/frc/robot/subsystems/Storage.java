/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libs.sensorsIMPL.Pixy;

public class Storage extends SubsystemBase {

  private VictorSPX belt, joint, intake;


  private DigitalInput IR_Internal;
  private Ultrasonic ultrasonic;
  private Pixy pixy;

  public boolean[] balls =  {false, false, false, false, false};

  public Storage() {
    pixy = new Pixy();

    belt = new VictorSPX(Constants.Ports.Motors.STORAGE_BELT);
    joint = new VictorSPX(Constants.Ports.Motors.COLLECTOR_JOINT);
    intake = new VictorSPX(Constants.Ports.Motors.COLLECTOR_INTAKE);

    IR_Internal = new DigitalInput(Constants.Ports.Sensors.STORAGE_OPTIC);
    ultrasonic = new  Ultrasonic(Constants.Ports.Sensors.STORAGE_ULTRASONIC_PING,
                                 Constants.Ports.Sensors.STORAGE_ULTRASONIC_ECHO);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void MoveBelt(double speed){
    belt.set(ControlMode.PercentOutput, speed * Constants.storage_belt_max_speed);
  }

  public void MoveJoint(double speed){
    joint.set(ControlMode.PercentOutput, speed);
  }

  public void pullPowerCell(){
    intake.set(ControlMode.PercentOutput, Constants.intake_speed);
  }

  public void stopPowerCell(){
    intake.set(ControlMode.PercentOutput, 0);
  }

  public double getUltrasonicRange(){
    return ultrasonic.getRangeMM();
  }

  public boolean getIRValue(){
    return IR_Internal.get();
  }

  public boolean getUltrasonicBool() {
    // Ajustar valor de verificação
    if (ultrasonic.getRangeMM() < 100) {
      return true;
    } else {
      return false;
    }
  }

  public double getArea(){
    return pixy.getBiggestBlock().getY() * pixy.getBiggestBlock().getX();
  }

}
