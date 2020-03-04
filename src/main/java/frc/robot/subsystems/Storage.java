//----------------------------------------------------------------------------/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
//----------------------------------------------------------------------------/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Storage extends SubsystemBase {

  private VictorSPX belt, joint, intake;

  private NetworkTableEntry entryS0, entryS1, entryB1, entryB2, entryB3, entryB4, entryB5;
  private ShuffleboardTab tabColetor;
  // S0's the first position in storage.
  // S1's the second position in storage.
  private DigitalInput OP_S0, OP_S1;
  //private Pixy pixy;

  public boolean[] balls =  {false, false, false, false, false};

  public boolean shoot = false;

  public Storage() {
    belt = new VictorSPX(Constants.Ports.Motors.STORAGE_BELT);
    joint = new VictorSPX(Constants.Ports.Motors.COLLECTOR_JOINT);
    intake = new VictorSPX(Constants.Ports.Motors.COLLECTOR_INTAKE);
    belt.setInverted(true);

    OP_S0 = new DigitalInput(Constants.Ports.Sensors.STORAGE_OPTIC_S0);
    OP_S1 = new DigitalInput(Constants.Ports.Sensors.STORAGE_OPTIC_S1);

    tabColetor = Shuffleboard.getTab("Coletor");
    entryS0 = tabColetor.add("S0", false).getEntry();
    entryS1 = tabColetor.add("S1", false).getEntry();
    entryB1 = tabColetor.add("B1", false).getEntry();
    entryB2 = tabColetor.add("B2", false).getEntry();
    entryB3 = tabColetor.add("B3", false).getEntry();
    entryB4 = tabColetor.add("B4", false).getEntry();
    entryB5 = tabColetor.add("B5", false).getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    entryS0.setBoolean(this.getOPS0());
    entryS1.setBoolean(this.getOPS1());

    entryB1.setBoolean(balls[0]);
    entryB2.setBoolean(balls[1]);
    entryB3.setBoolean(balls[2]);
    entryB4.setBoolean(balls[3]);
    entryB5.setBoolean(balls[4]);
    /*
    if (this.getBallArea() > Constants.STORAGE_MIN_PIXY_AREA) {
      this.pullPowerCell();
    } else {
      this.stopPowerCell();
    }
    */
  }

  public void MoveBelt(){
    belt.set(ControlMode.PercentOutput, Constants.STORAGE_BELT_MAX_SPEED);
  }

  public void stopBelt() {
    belt.set(ControlMode.PercentOutput, 0);
  }

  public void MoveJoint(double speed){
    joint.set(ControlMode.PercentOutput, speed);
  }

  public void pullPowerCell(){
    intake.set(ControlMode.PercentOutput, Constants.INTAKE_SPEED);
  }

  public void reversePowerCell() {
    intake.set(ControlMode.PercentOutput, -Constants.INTAKE_SPEED);
  }

  public void stopPowerCell(){
    intake.set(ControlMode.PercentOutput, 0);
  }

  public boolean getOPS0(){
    return OP_S0.get();
  }

  public boolean getOPS1(){
    return !OP_S1.get();
  }

  public double getBallArea() {
  //return pixy.getBiggestBlock().getX() * pixy.getBiggestBlock().getY();
   return 0;
  }
}