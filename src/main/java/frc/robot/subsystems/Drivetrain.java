/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class Drivetrain extends PIDSubsystem {

  private WPI_VictorSPX leftFront, leftBack, rightFront, rightBack;
  private SpeedControllerGroup left, right;
  private DifferentialDrive m_drive;
  private AHRS m_navX;
  //ENCODERS
  //***
  Encoder encoderLeft, encoderRight;

  public Drivetrain(AHRS navX) {
    super(new PIDController(Constants.drive_kP, Constants.drive_kI, Constants.drive_kD));

    m_navX = navX;

    encoderLeft = new Encoder(Constants.Ports.Sensors.DRIVE_ENC_LEFT_A, Constants.Ports.Sensors.DRIVE_ENC_LEFT_B, false);
    encoderLeft.setDistancePerPulse(Math.PI * 4 * 2.54 / 360.0);
    encoderLeft.setMinRate(1.0);
    encoderLeft.setSamplesToAverage(5);

    encoderRight = new Encoder(Constants.Ports.Sensors.DRIVE_ENC_RIGHT_A, Constants.Ports.Sensors.DRIVE_ENC_RIGHT_B, false);
    encoderRight.setDistancePerPulse(Math.PI * 4 * 2.54 / 360.0);
    encoderRight.setMinRate(1.0);
    encoderRight.setSamplesToAverage(5);

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
    return m_navX.getYaw();
  }
}
