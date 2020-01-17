/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.libs.sensors.NavX;
import edu.wpi.first.wpilibj.SerialPort;

public class Drivetrain extends PIDSubsystem {
  /**
   * Creates a new Drivetrain.
   */
  Spark LeftFront, LeftBack, RightFront, RightBack;

  private final SpeedControllerGroup m_leftMotors, m_rightMotors; 
  private final DifferentialDrive m_drive;

  private final AHRS m_navx;

  public Drivetrain() {
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));
      
      LeftFront = new Spark(Constants.Drive.LEFT_FRONT.getPort());
      LeftBack = new Spark(Constants.Drive.LEFT_BACK.getPort());
      RightFront = new Spark(Constants.Drive.RIGHT_FRONT.getPort());
      RightBack = new Spark(Constants.Drive.RIGHT_BACK.getPort());

      m_leftMotors = new SpeedControllerGroup(LeftFront, LeftBack);
      m_rightMotors = new SpeedControllerGroup(RightFront, RightBack);

      m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

      m_navx = new AHRS(SerialPort.Port.kMXP);
  }

  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    m_drive.arcadeDrive(0, output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_navx.getYaw();
  }
}
