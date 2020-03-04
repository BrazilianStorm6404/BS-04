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
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class Drivetrain extends PIDSubsystem {

  private Spark _leftFront, _leftBack, _rightBack, _rightFront;
  private SpeedControllerGroup _left, _right;
  private DifferentialDrive m_drive;
  private AHRS _navX;

  private final DifferentialDriveOdometry m_odometry;
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

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    drivetrainTab = Shuffleboard.getTab("Tração");
    encoderLeftEntry = drivetrainTab.add("Encoder Esquerdo", 0.0).getEntry();
    encoderRightEntry = drivetrainTab.add("Encoder Direito", 0.0).getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    encoderLeftEntry.forceSetDouble(this.getEncoderLeft());
    encoderRightEntry.forceSetDouble(this.getEncoderRight());

    // Atualização de odometria para PathWeaver
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), encoderLeft.getDistance(),
                      encoderRight.getDistance());
  }

  // Funções utilitárias para PathWeaver
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    _navX.reset();
  }


  public void tankDriveVolts(double leftVolts, double rightVolts) {
    _left.setVoltage(leftVolts);
    _right.setVoltage(-rightVolts);
    m_drive.feed();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public double getEncoderRight() {
    return encoderRight.getDistance();
  }

  public double getEncoderLeft() {
    return encoderLeft.getDistance();
  }

  public double getAverageEncoderDistance() {
    return (encoderLeft.getDistance() + encoderRight.getDistance()) / 2.0;
  }

  public double getHeading() {
    return Math.IEEEremainder(_navX.getYaw(), 360);
  }

  public double getTurnRate() {
    return _navX.getRate();
  }

  public void resetEncoders() {
    encoderLeft.reset();
    encoderRight.reset();
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
