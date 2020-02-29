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

public class Storage extends SubsystemBase {

  private int powerCells = 0;

  private WPI_VictorSPX belt;

  private DigitalInput IR_intake_detector,IR_intake_verifier;

  public Storage() {
    belt = new WPI_VictorSPX(Constants.Ports.Motors.STORAGE_BELT);

    IR_intake_detector = new DigitalInput(Constants.Ports.Sensors.STORAGE_OPTIC);
    IR_intake_verifier = new DigitalInput(Constants.Ports.Sensors.INTAKE_OPTIC);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void MoveBelt(double speed){
    belt.set(ControlMode.PercentOutput, speed * Constants.STORAGE_BELT_SPEED);
  }

  public boolean getIRDetectorValue(){
    return IR_intake_detector.get();
  }

  public boolean getIRVerifierValue() {
    return IR_intake_verifier.get();
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
