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

  // MOTORS
  WPI_VictorSPX HorizontalTurner;
  WPI_VictorSPX VerticalTurner;


  /**
   * Creates a new Tracker.
   */
  public Tracker() {
    // LIMITS SWITCHES

    // MOTORS
    VerticalTurner = new WPI_VictorSPX(Constants.Motors.VERTICAL_TURNER.getPort());
    HorizontalTurner = new WPI_VictorSPX(Constants.Motors.HORIZONTAL_TURNER.getPort());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Command for turning the launcher vertically.
   * @param direction if true turns to the right, if false turns to the left.
   */
  public void turnVertical(boolean direction) {
    VerticalTurner.set(direction ? 0.7 : -0.7);
  } 

  /**
   * Command for turning the launcher horizontally.
   * @param direction if true turns up, if false turns down.
   */
  public void turnHorizontal(boolean direction) {
    HorizontalTurner.set(direction ? 0.7 : -0.7);
  } 
}
