//----------------------------------------------------------------------------/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
//----------------------------------------------------------------------------/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Storage extends SubsystemBase {

  private int powerCells = 3;

  private ShuffleboardTab tabStorage;

  private NetworkTableEntry IR_detector, IR_verifier, PowerCellCount, IR_shooter;

  private WPI_VictorSPX belt;

  private DigitalInput IR_intake_detector,IR_intake_verifier, IR_intake_shooter;

  public boolean shoot = false;

  public Storage() {
    belt = new WPI_VictorSPX(Constants.Ports.Motors.STORAGE_BELT);

    IR_intake_detector = new DigitalInput(Constants.Ports.Sensors.STORAGE_OPTIC);
    IR_intake_verifier = new DigitalInput(Constants.Ports.Sensors.INTAKE_OPTIC);
    IR_intake_shooter = new DigitalInput(Constants.Ports.Sensors.SHOOTER_OPTIC);

    tabStorage = Shuffleboard.getTab("Storage");
    IR_detector = tabStorage.add("IR detector",false).getEntry();
    IR_verifier = tabStorage.add("IR verifier", false).getEntry();
    IR_shooter = tabStorage.add("IR shooter", false).getEntry();
    PowerCellCount = tabStorage.add("Contagem de power cells", 0).getEntry();
  }

  @Override
  public void periodic() {
    IR_detector.setBoolean(this.getIRDetectorValue());
    IR_shooter.setBoolean(this.getIRShooterValue());
    IR_verifier.setBoolean(this.getIRVerifierValue());
    PowerCellCount.forceSetNumber(this.getPowerCellCount());
  }

  public void MoveBelt(double speed){
    belt.set(speed);
  }

  public boolean getIRDetectorValue(){
    return !IR_intake_detector.get();
  }

  public boolean getIRVerifierValue() {
    return IR_intake_verifier.get();
  }

  public boolean getIRShooterValue() {
    return IR_intake_shooter.get();
  }

  public void addPowerCells() {
    powerCells++;
  }

  public int getPowerCellCount() {
    return powerCells;
  }

  public void removePowerCells() {
    powerCells--;
  }
}