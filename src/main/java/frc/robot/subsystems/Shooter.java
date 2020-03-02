/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

	private Spark angle;
	private VictorSP belt;
	private VictorSPX shoot;

	private ADXRS450_Gyro gyro;
	private DigitalInput limitHigh;
	private DigitalInput limitLow;

	public Shooter() {
		angle = new Spark(Constants.Ports.Motors.SHOOTER_ANGLE);
		belt = new VictorSP(Constants.Ports.Motors.SHOOTER_BELT);
		shoot = new VictorSPX(Constants.Ports.Motors.SHOOTER_SHOOT);

		gyro = new ADXRS450_Gyro();
		limitHigh = new DigitalInput(Constants.Ports.Sensors.SHOOTER_LIMIT_HIGH);
		limitLow = new DigitalInput(Constants.Ports.Sensors.SHOOTER_LIMIT_LOW);
		belt.setInverted(true);
	}

	@Override
	public void periodic() {
		if (limitLow.get()) {
			gyro.reset();
		}
	}

	public void Shoot() {
		shoot.set(ControlMode.PercentOutput, Constants.SHOOTING_SPEED);
		belt.set(Constants.SHOOTER_BELT_SPEED);
	}

	public void moveUp() {
		double speed = Constants.SHOOTER_ANGLE_SPEED;

		angle.set(speed);
	}

	public void moveDown() {
		double speed = Constants.SHOOTER_ANGLE_SPEED * -1;

		angle.set(speed);
	}

	public void stopMoving() {
		angle.set(0);
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
