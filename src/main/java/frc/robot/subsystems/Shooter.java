/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

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

	//#region Sensors, controllers & etc.
	private WPI_VictorSPX angle, shoot;
	private VictorSP belt;

	private NetworkTableEntry entryGyro, entryLimitHigh, entryLimitLow;
	private ShuffleboardTab tabShooter;

	private ADXRS450_Gyro gyro;
	private DigitalInput limitHigh;
	private DigitalInput limitLow;
	//#endregion

	public Shooter() {
		// Motors
		angle = new WPI_VictorSPX(Constants.Ports.Motors.SHOOTER_ANGLE);
		shoot = new WPI_VictorSPX(Constants.Ports.Motors.SHOOTER_SHOOT);
		belt = new VictorSP(Constants.Ports.Motors.SHOOTER_BELT);

		// Sensors
		gyro = new ADXRS450_Gyro();
		limitHigh = new DigitalInput(Constants.Ports.Sensors.SHOOTER_LIMIT_HIGH);
		limitLow = new DigitalInput(Constants.Ports.Sensors.SHOOTER_LIMIT_LOW);

		// Set the values to inverted
		belt.setInverted(true);
		angle.setInverted(true);

		// Shuffleboard
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

		// Set values in shuffleboard
		entryLimitHigh.setBoolean(this.getLimitHigh());
		entryLimitLow.setBoolean(this.getLimitLow());
		entryGyro.forceSetDouble(this.getGyroAngle());
	}

	//#region SHOOTER
	/**
	 * Set the values to shoot the shooter
	 */
	public void Shoot() {
		shoot.set(-Constants.SHOOTING_SPEED);
		belt.set(Constants.SHOOTER_BELT_SPEED);
	}

	/**
	 * Set all motor values to 0
	 */
	public void Stop() {
		stopMoving();
		stopShooting();
	}

	/**
	 * set the angle to the constant defined in Constants
	 */
	public void moveUp() {
		angle.set(Constants.SHOOTER_ANGLE_SPEED);
	}

	/**
	 * set the angle to the constant defined in Constants in negative
	 */
	public void moveDown() {
		angle.set(-Constants.SHOOTER_ANGLE_SPEED);
	}

	/**
	 * set the angle to 0
	 */
	public void stopMoving() {
		angle.set(0);
	}

	/**
	 * set the shoot and belt to 0
	 */
	public void stopShooting() {
		shoot.set(0);
		stopBelt();
	}

	/**
	 * set the belt to 0
	 */
	public void stopBelt() {
		belt.set(0);
	}

	/**
	 * set the belt to the constant defined in Constants
	 */
	public void moveBelt() {
		belt.set(Constants.SHOOTER_BELT_SPEED);
	}

	//#endregion

	//#region ANGLES

	/**
	 * Gets the angle of the gyro
	 * @return the angle of the gyro
	 */
	public double getGyroAngle() {
		return gyro.getAngle();
	}

	/**
	 * Gets the value of the lower limit
	 * @return the value of the lower limit
	 */
	public boolean getLimitLow() {
		return limitLow.get();
	}

	/**
	 * Gets the value of the upper limit
	 * @return the value of the upper limit
	 */
	public boolean getLimitHigh() {
		return limitHigh.get();
	}

	//#endregion

}
