/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Storage;

public class ControlStorage extends CommandBase {

	private Storage m_storage;
	private boolean s0, lastS0 = false;
	private boolean s1, lastS1 = false;
	private boolean pulling = false;
	private int i = 0;
	private boolean considerGap = false;

	public ControlStorage(Storage Storage) {
		addRequirements(Storage);
		m_storage = Storage;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		s0 = m_storage.getIRDetectorValue();
		s1 = m_storage.getIRVerifierValue();

		if (s0) {
			pulling = true;
			if (!m_storage.balls[0]) {
				if (!(s1 && (!lastS1))) {
					m_storage.balls[i] = true;
					i++;
					considerGap = false;
				}
			} else {
				considerGap = true;
			}
		} else {
			pulling = false;
			if (considerGap) {
				if (s0 && (!lastS0)) {
					m_storage.MoveBelt(0);
					m_storage.balls[i]  = true;
					i++;
					considerGap = false;
				}
			}
		}

		if (pulling) {
			m_storage.MoveBelt(Constants.STORAGE_BELT_SPEED);
		} else {
			m_storage.MoveBelt(0.0);
		}

		s0 = m_storage.getIRDetectorValue();
		s1 = m_storage.getIRVerifierValue();
		// m_storage.MoveBelt(Constants.STORAGE_BELT_SPEED);
		/*
		if(intake) {
			m_storage.MoveBelt(Constants.STORAGE_BELT_SPEED);
		} else if(!intake && verifier) {
			m_storage.MoveBelt(0.0);
			m_storage.addPowerCells();
		}
		*/
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}

}
