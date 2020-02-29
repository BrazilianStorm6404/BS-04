/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libs.sensorsimpl.Pixy;

public class Storage extends SubsystemBase {

  private WPI_VictorSPX belt, joint;

  private DigitalInput IR_intake_detector,IR_intake_verifier;
  private Pixy pixy;

  public boolean[] balls =  {false, false, false, false, false};

  public Storage() {
    pixy = new Pixy();

    belt = new WPI_VictorSPX(Constants.Ports.Motors.STORAGE_BELT);
    joint = new WPI_VictorSPX(Constants.Ports.Motors.COLLECTOR_JOINT);

    IR_intake_detector = new DigitalInput(Constants.Ports.Sensors.STORAGE_OPTIC);
    IR_intake_verifier = new DigitalInput(Constants.Ports.Sensors.INTAKE_OPTIC);
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

  public boolean getIRDetectorValue(){
    return IR_intake_detector.get();
  }

  public boolean getIRVerifierValue() {
    return IR_intake_verifier.get();
  }

  public double getArea(){
    return pixy.getBiggestBlock().getY() * pixy.getBiggestBlock().getX();
  }

}
