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

  // S0's the first position in storage.
  // S1's the second position in storage.
  private DigitalInput OP_S0, OP_S1;
  private Pixy pixy;

  public boolean[] balls =  {false, false, false, false, false};

  public Storage() {
    pixy = new Pixy();

    belt = new VictorSPX(Constants.Ports.Motors.STORAGE_BELT);
    joint = new VictorSPX(Constants.Ports.Motors.COLLECTOR_JOINT);
    intake = new VictorSPX(Constants.Ports.Motors.COLLECTOR_INTAKE);

    OP_S0 = new DigitalInput(Constants.Ports.Sensors.STORAGE_OPTIC_S0);
    OP_S1 = new DigitalInput(Constants.Ports.Sensors.STORAGE_OPTIC_S1);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void MoveBelt(double speed){
    belt.set(ControlMode.PercentOutput, speed * Constants.STORAGE_BELT_MAX_SPEED);
  }

  public void MoveJoint(double speed){
    joint.set(ControlMode.PercentOutput, speed);
  }

  public void pullPowerCell(){
    intake.set(ControlMode.PercentOutput, Constants.INTAKE_SPEED);
  }

  public void stopPowerCell(){
    intake.set(ControlMode.PercentOutput, 0);
  }

  public boolean getOPS0(){
    return OP_S0.get();
  }

  public boolean getOPS1(){
    return OP_S1.get();
  }


  public double getArea(){
    return pixy.getBiggestBlock().getY() * pixy.getBiggestBlock().getX();
  }

}
