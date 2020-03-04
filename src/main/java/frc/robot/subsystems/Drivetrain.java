/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class Drivetrain extends PIDSubsystem {

  private Spark _leftFront, _leftBack, _rightBack, _rightFront;
  private SpeedControllerGroup _left, _right;
  private DifferentialDrive m_drive;
  private AHRS _navX;
  //ENCODERS
  Encoder encoderLeft, encoderRight;

  //SHUFFLEBOARD
  private ShuffleboardTab drivetrainTab;
  private NetworkTableEntry encoderLeftEntry, encoderRightEntry;
  
  public Drivetrain(AHRS navX) {
    super(new PIDController(Constants.DRIVE_kP, Constants.DRIVE_kI, Constants.DRIVE_kD));

    this._navX = navX;

    encoderLeft = new Encoder(Constants.Ports.Sensors.DRIVE_ENC_LEFT_A, Constants.Ports.Sensors.DRIVE_ENC_LEFT_B, false);
    encoderLeft.setDistancePerPulse(Math.PI * 4 * 2.54 / 360.0);
    encoderLeft.setMinRate(1.0);
    encoderLeft.setSamplesToAverage(5);

    encoderRight = new Encoder(Constants.Ports.Sensors.DRIVE_ENC_RIGHT_A, Constants.Ports.Sensors.DRIVE_ENC_RIGHT_B, false);
    encoderRight.setDistancePerPulse(Math.PI * 4 * 2.54 / 360.0);
    encoderRight.setMinRate(1.0);
    encoderRight.setSamplesToAverage(5);

    _leftFront = new Spark(Constants.Ports.Motors.DRIVE_LEFT_FRONT);
    _leftBack = new Spark(Constants.Ports.Motors.DRIVE_LEFT_BACK);
    _rightFront = new Spark(Constants.Ports.Motors.DRIVE_RIGHT_FRONT);
    _rightBack = new Spark(Constants.Ports.Motors.DRIVE_RIGHT_BACK);

    _left = new SpeedControllerGroup(_leftFront, _leftBack);
    _right = new SpeedControllerGroup(_rightFront, _rightBack);

    m_drive = new DifferentialDrive(_left, _right);

    drivetrainTab = Shuffleboard.getTab("Tração");
    encoderLeftEntry = drivetrainTab.add("Encoder Esquerdo", 0.0).getEntry();
    encoderRightEntry = drivetrainTab.add("Encoder Direito", 0.0).getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    encoderLeftEntry.forceSetDouble(this.getEncoderLeft());
    encoderRightEntry.forceSetDouble(this.getEncoderRight());
  }

  public double getEncoderRight() {
    return encoderRight.getDistance();
  }

  public double getEncoderLeft() {
    return encoderLeft.getDistance();
  }

  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  public void stop(){
    m_drive.arcadeDrive(0, 0);
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    Shuffleboard.getTab("DriveTrain").addNumber("PID Output", () -> output);
  }

  @Override
  protected double getMeasurement() {
    return _navX.getYaw();
  }
}
