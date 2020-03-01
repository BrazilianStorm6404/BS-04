/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.libs.auto.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class Straight extends PIDCommand {
  /**
   * Creates a new Straigth.
   */
  boolean killed = false;
  /**
   * Create a new Command to drive straight using a NavX sensor
   */
  public Straight(AHRS navX, Drivetrain drivetrain, double setpoint) {
    super(
        new PIDController(Constants.DRIVE_kP, Constants.DRIVE_kI, Constants.DRIVE_kD),
        () -> {
          return navX.getAngle();
        },
        () -> {
          return setpoint;
        },
        output -> {
          if (output != 0) {
            drivetrain.arcadeDrive(0, output);
          }
        });
  }
  /**
   * Use to stop the PIDController
   */
  public void kill() {
    killed = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return killed;
  }
}