/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Storage;

public class ControlStorage extends CommandBase {

	private Storage _storage;
	int i = 0;

	boolean s0 = false;
	boolean s1 = false;
	boolean lastS0 = false;
	boolean lastS1 = false;
	boolean pulling = false;
	boolean considerGap = false;

	public ControlStorage(Storage m_storage) {
		addRequirements(m_storage);

		_storage = m_storage;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		s0 = _storage.getOPS0();
		s1 = _storage.getOPS1();

		if (s0) {
			pulling = true;
			if (!_storage.balls[0]) {
				if (!(s1 && (!lastS1))) {
					_storage.balls[i] = true;
					i++;
					considerGap = false;
					pulling = false;
				}
			} else {
				considerGap = true;
			}
		} else {
			pulling = false;
			if (considerGap) {
				if (s0 && (!lastS0)) {
					_storage.MoveBelt();
					_storage.balls[i] = true;
					i++;
					considerGap = false;
					pulling = false;
				}
			}
		}

		if (pulling) {
			_storage.MoveBelt();
		} else {
			_storage.stopBelt();
		}

		lastS0 = _storage.getOPS0();
		lastS1 = _storage.getOPS1();
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
