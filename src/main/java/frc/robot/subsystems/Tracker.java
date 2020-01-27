/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Tracker extends SubsystemBase {

  //#region INSTANTIATION
  // MOTORS
  WPI_VictorSPX VerticalTurner;
  WPI_VictorSPX Shooter;
  //
  //#endregion

  //#region CONSTRUCTOR
  /**
   * Creates a new Tracker.
   */
  public Tracker() {

    // MOTORS
    VerticalTurner = new WPI_VictorSPX(Constants.Motors.VERTICAL_TURNER.getPortCAN());
    Shooter = new WPI_VictorSPX(Constants.Motors.SHOOTER.getPortCAN());
  }
  //#endregion

  //#region PERIODIC
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  //#endregion

  //#region TURN VERTICAL
  /**
   * Command for turning the launcher vertically.
   * @param direction if true turns to the right, if false turns to the left.
   */
  public void turnVertical(boolean direction) {
    VerticalTurner.set(direction ? 0.7 : -0.7);
  } 
  //#endregion

  //#region SHOOT
  public void shoot() {
    Shooter.set(Constants.Motors.getShootingSpeed());
  } 
  //#endregion
}
