/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.libs.auto.drive;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.libs.sensors.NavX;
import frc.robot.subsystems.DriveTrain;

public class Straigth extends PIDCommand {
  /**
   * Creates a new Straigth.
   */
  boolean killed = false;
  /**
   * Create a new Command to drive straight using a NavX sensor
   */
  public Straigth(NavX navX, DriveTrain drivetrain, double setpoint) {
    super(
        new PIDController(Constants.kP, Constants.kI, Constants.kD),
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