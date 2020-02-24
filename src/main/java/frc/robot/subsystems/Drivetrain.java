/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class Drivetrain extends PIDSubsystem {

  private WPI_VictorSPX leftFront, leftBack, rightFront, rightBack;
  private SpeedControllerGroup left, right;
  private final DifferentialDrive m_drive;
  private AHRS navX;

  public Drivetrain(AHRS navX) {
    super(new PIDController(Constants.kP, Constants.kI, Constants.kD));

    this.navX = navX;

    leftFront = new WPI_VictorSPX(Constants.Ports.Motors.DRIVE_LEFT_FRONT);
    leftBack = new WPI_VictorSPX(Constants.Ports.Motors.DRIVE_LEFT_BACK);
    rightFront = new WPI_VictorSPX(Constants.Ports.Motors.DRIVE_RIGHT_FRONT);
    rightBack = new WPI_VictorSPX(Constants.Ports.Motors.DRIVE_RIGHT_BACK);

    left = new SpeedControllerGroup(leftFront, leftBack);
    right = new SpeedControllerGroup(rightFront, rightBack);

    m_drive = new DifferentialDrive(left, right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  public void stop(){
    m_drive.arcadeDrive(0, 0);
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    Shuffleboard.getTab("DriveTarin").addNumber("PID Output", () -> output);

  }

  @Override
  protected double getMeasurement() {
    return navX.getYaw();
  }
}
