/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ShooterAlign extends PIDCommand {

	private static boolean finished = false;

	public ShooterAlign(Shooter m_Shooter) {
		super(
			// The controller that the command will use
			new PIDController(Constants.SHOOTER_kP, Constants.SHOOTER_kI, Constants.SHOOTER_kD),
			// This should return the measurement
			() -> m_Shooter.getGyroAngle(),
			// This should return the setpoint (can also be a constant)
			() -> {
				return treatDistance(Shuffleboard.getTab("Vision").add("Distance", 0).getEntry().getDouble(0));
			},
			output -> {
				m_Shooter.moveUp();
				if (output < 0.05) {
					finished = true;
				}
			}
		);
		
		addRequirements(m_Shooter);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return finished;
	}

	public static double treatDistance(double dist) { 
		double g = 9.8;
		double h = 1.75;
		double vo = 25;

		double vo2 = Math.pow(vo, 2);
		double dist2 = Math.pow(dist, 2);

		double t1_arctg = dist + Math.sqrt(dist2 - ((2 * g * dist2) / vo2) * (((g * dist2) / (2 * vo2)) + h));
		double t2_arctg = (g * dist2) / vo2;
		double atan = t1_arctg / t2_arctg;

		return Math.atan(atan);
	}
}
