/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

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
        new WPI_VictorSPX(Constants.Motors.DRIVE_LEFT_FRONT.getPortCAN()), 
        new WPI_VictorSPX(Constants.Motors.DRIVE_LEFT_BACK.getPortCAN())),
      // RIGHT SPEED CONTROLLER
      new SpeedControllerGroup(
        new WPI_VictorSPX(Constants.Motors.DRIVE_RIGHT_BACK.getPortCAN()),
      new WPI_VictorSPX(Constants.Motors.DRIVE_RIGHT_FRONT.getPortCAN()))
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
