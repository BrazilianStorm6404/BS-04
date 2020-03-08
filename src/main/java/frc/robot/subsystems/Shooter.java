/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

	private WPI_VictorSPX angle, shoot;
	private VictorSP belt;

	private NetworkTableEntry entryGyro, entryLimitHigh, entryLimitLow;
	private ShuffleboardTab tabShooter;

	private ADXRS450_Gyro gyro;
	private DigitalInput limitHigh;
	private DigitalInput limitLow;

	public Shooter() {
		angle = new WPI_VictorSPX(Constants.Ports.Motors.SHOOTER_ANGLE);
		belt = new VictorSP(Constants.Ports.Motors.SHOOTER_BELT);
		shoot = new WPI_VictorSPX(Constants.Ports.Motors.SHOOTER_SHOOT);

		gyro = new ADXRS450_Gyro();
		limitHigh = new DigitalInput(Constants.Ports.Sensors.SHOOTER_LIMIT_HIGH);
		limitLow = new DigitalInput(Constants.Ports.Sensors.SHOOTER_LIMIT_LOW);
		belt.setInverted(true);
		angle.setInverted(true);

		tabShooter = Shuffleboard.getTab("Shooter");
		entryLimitHigh = tabShooter.add("Limit Cima", false).getEntry();
		entryLimitLow = tabShooter.add("Limit Baixo", false).getEntry();
		entryGyro = tabShooter.add("Gyro", 0.0).getEntry();
	}

	@Override
	public void periodic() {
		if (limitLow.get()) {
			gyro.reset();
		}

		entryLimitHigh.setBoolean(this.getLimitHigh());
		entryLimitLow.setBoolean(this.getLimitLow());
		entryGyro.forceSetDouble(this.getGyroAngle());
	}

	public boolean getLimitLow() {
		return limitLow.get();
	}

	public boolean getLimitHigh() {
		return limitHigh.get();
	}

	public void Shoot() {
		shoot.set( Constants.SHOOTING_SPEED);
		belt.set(Constants.SHOOTER_BELT_SPEED);
	}

	public void moveUp() {
		angle.set(Constants.SHOOTER_ANGLE_SPEED);
	}

	public void moveDown() {
		double speed = Constants.SHOOTER_ANGLE_SPEED * -1;

		angle.set(ControlMode.PercentOutput, speed);
	}

	public void stopMoving() {
		angle.set(ControlMode.PercentOutput, 0);
	}
	
	public void stopShooting() {
		shoot.set(ControlMode.PercentOutput, 0);
	}

	public void stopBelt() {
		belt.set(0);
	}

	public void moveBelt() {
		belt.set(Constants.SHOOTER_BELT_SPEED);
	}

	public double getGyroAngle() {
		return gyro.getAngle();
	}

}
