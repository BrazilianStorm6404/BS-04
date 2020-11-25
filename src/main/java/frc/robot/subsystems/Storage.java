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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Storage extends SubsystemBase {

  //#region Sensors, controllers & etc.
  public int powerCells = 3;

  private ShuffleboardTab tabStorage;

  private NetworkTableEntry IR_detector, IR_verifier, PowerCellCount, IR_shooter;

  private WPI_VictorSPX belt;

  private DigitalInput IR_intake_detector,IR_intake_verifier, IR_intake_shooter;

  public boolean shoot = false;

  public boolean[] balls = {false, false, false, false, false};
  //#endregion

  public Storage() {
    // Motor
    belt = new WPI_VictorSPX(Constants.Ports.Motors.STORAGE_BELT);

    // Sensores
    IR_intake_detector = new DigitalInput(Constants.Ports.Sensors.STORAGE_OPTIC);
    IR_intake_verifier = new DigitalInput(Constants.Ports.Sensors.INTAKE_OPTIC);
    IR_intake_shooter = new DigitalInput(Constants.Ports.Sensors.SHOOTER_OPTIC);

    //Shuffle
    tabStorage = Shuffleboard.getTab("Storage");
    IR_detector = tabStorage.add("IR detector",false).getEntry();
    IR_verifier = tabStorage.add("IR verifier", false).getEntry();
    IR_shooter = tabStorage.add("IR shooter", false).getEntry();
    PowerCellCount = tabStorage.add("Power cell count", 0.0).getEntry();
  }

  @Override
  public void periodic() {
    // Shuffle
    IR_detector.setBoolean(this.getIRDetectorValue());
    IR_shooter.setBoolean(this.getIRShooterValue());
    IR_verifier.setBoolean(this.getIRVerifierValue());
    PowerCellCount.setNumber(powerCells);
  }

  //#region BELT

  /**
   * movimenta o motor do coneveyor/belt de acordo com a velocidade
   * @param speed velocidade a ser usada
   */
  public void MoveBelt(double speed){
    belt.set(speed);
  }
  //#endregion

  //#region SENSORS

  /**
   * obtém o valor do sensor infravermelho (IR) da entrada do conveyor/storage
   * @return o valor obtido pelo sensor
   */
  public boolean getIRDetectorValue(){
    return IR_intake_detector.get();
  }

  /**
   * obtém o valor do sensor infravermelho (IR) no meio do conveyor/storage
   * @return o valor obtido pelo sensor
   */
  public boolean getIRVerifierValue() {
    return !IR_intake_verifier.get();
  }

  /**
   * obtém o valor do sensor infravermelho (IR) no topo do conveyor/storage
   * @return o valor obtido pelo sensor
   */
  public boolean getIRShooterValue() {
    return IR_intake_shooter.get();
  }
  //#endregion

  //#region POWER CELLS

  /**
   * adiciona power cells ao controlador interno
   */
  public void addPowerCells() {
    powerCells++;
  }

  /**
   * obtém a quantidade de power cells no robô
   * @return
   */
  public int getPowerCellCount() {
    return powerCells;
  }

  /**
   * remove power cells do robô
   */
  public void removePowerCells() {
    powerCells--;
  }
  //#endregion
}