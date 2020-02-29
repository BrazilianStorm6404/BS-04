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

	private int PowerCellInitialCount;

	private boolean intake = false;
	private boolean verifier = false;

	public ControlStorage(Storage Storage) {
		addRequirements(Storage);
		m_storage = Storage;
		PowerCellInitialCount = Storage.getPowerCellCount();
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		intake = m_storage.getIRDetectorValue();
		verifier = m_storage.getIRVerifierValue();

		if(intake) {
			m_storage.MoveBelt(Constants.STORAGE_BELT_SPEED);
		} else if(!intake && verifier) {
			m_storage.MoveBelt(0.0);
			m_storage.addPowerCells();
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return PowerCellInitialCount != m_storage.getPowerCellCount();
	}
}
