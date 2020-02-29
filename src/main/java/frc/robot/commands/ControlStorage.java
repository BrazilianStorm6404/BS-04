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

	boolean ultra = false;
	boolean op = false;
	boolean lastUltra = false;
	boolean lastOp = false;
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
		ultra = _storage.getUltrasonicBool();
		op = _storage.getIRValue();

		if (ultra) {
			pulling = true;
			if (_storage.balls[1] == false) {
				if (op) {
					_storage.MoveBelt(0);
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
				if (op && (!lastOp)) {
					_storage.MoveBelt(0);
					_storage.balls[i] = true;
					i++;
					considerGap = false;
					pulling = false;
				}
			}
		}

		lastUltra = _storage.getUltrasonicBool();
		lastOp = _storage.getIRValue();
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
