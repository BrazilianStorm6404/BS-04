/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class DriveTrain extends SubsystemBase {
//#region INSTANTIATION
private final DifferentialDrive m_drive;

//#endregion

  //#region CONSTRUCTOR
  /**
  * Creates a new Drivetrain.
  */
  public DriveTrain() {

    m_drive = new DifferentialDrive(
      // LEFT SPEED CONTROLLER
      new SpeedControllerGroup(
        new Spark(Constants.Motors.DRIVE_LEFT_FRONT.getPortPWM()), 
        new Spark(Constants.Motors.DRIVE_LEFT_BACK.getPortPWM())),
      // RIGHT SPEED CONTROLLER
      new SpeedControllerGroup(
        new Spark(Constants.Motors.DRIVE_RIGHT_BACK.getPortPWM()),
      new Spark(Constants.Motors.DRIVE_RIGHT_FRONT.getPortPWM()))
    );
  }
 //#endregion

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //#region ARCADE DRIVE
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }
  //#endregion
}
