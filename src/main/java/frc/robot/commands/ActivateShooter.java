/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ActivateShooter extends SequentialCommandGroup {
	/**
	 * Creates a new ShooterControl.
	 */
	public ActivateShooter(Shooter m_Shooter, Storage m_Storage, PowerDistributionPanel m_PDP) {

		super(new ShooterAlign(m_Shooter), new Shoot(m_Shooter, m_Storage, m_PDP));
	}
}
