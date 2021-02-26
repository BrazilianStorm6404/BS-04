// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class resetCommand extends CommandBase {

  private Drivetrain drivetrain;
  private boolean finished = false;
  Timer timer = new Timer();
  /** Creates a new resetCommand. */
  public resetCommand(Drivetrain m_Drivetrain) {
    addRequirements(m_Drivetrain);
    drivetrain= m_Drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetEncoders();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward = -0.9, turn = 0.8, correct = 0.4, time = timer.get();
    // 90 graus 0.5
    // ~1.73m/s

    // ------------- S H A L O M ---------------
    if (time < 0.6) {
      // D1
      drivetrain.arcadeDrive(forward, correct);
    } else if (time < 1.0) {
      // G1
      drivetrain.arcadeDrive(0, -turn);
    } else if (time < 1.6) {
      // D2
       drivetrain.arcadeDrive(forward, correct);
    } else if (time < 2.0) {
      // G2
      drivetrain.arcadeDrive(0, turn);
    } else if (time < 3.8) {
      // D3
      drivetrain.arcadeDrive(forward, correct-0.15);
    } else if (time < 4.2) {
      // G3
      drivetrain.arcadeDrive(0, turn);
    } else if (time < 4.8) {
      // D4
      drivetrain.arcadeDrive(forward, correct);
    } else if (time < 5.2) {
      // G4
      drivetrain.arcadeDrive(0, -turn);
    } else if (time < 5.8) {
      // D5
      drivetrain.arcadeDrive(forward, correct);
    } else if (time < 6.2) {
      // G5
      drivetrain.arcadeDrive(0, -turn);
    } else if (time < 6.8) {
      // D6
      drivetrain.arcadeDrive(forward, correct);
    } else if (time < 7.2) {
      // G6
      drivetrain.arcadeDrive(0, -turn);
    } else if (time < 7.8) {
      // D7
      drivetrain.arcadeDrive(forward, correct);
    } else if (time < 8.2) {
      // G7
      drivetrain.arcadeDrive(0, -turn);
    } else if (time < 8.8) {
      // D8
      drivetrain.arcadeDrive(forward, correct);
    } else if (time < 9.2) {
      // G8
      drivetrain.arcadeDrive(0, turn);
    } else if( time < 10.9) {
      // D9
      drivetrain.arcadeDrive(forward, correct-0.15);
    } else if (time < 11.3) {
      // G9
      drivetrain.arcadeDrive(0, turn);
    } else if (time < 11.9) {
      // D10
      drivetrain.arcadeDrive(forward, correct);
    } else if (time < 12.3) {
      // G10
      drivetrain.arcadeDrive(0, -turn);
    } else if (time < 13.9) {
      // D11
      drivetrain.arcadeDrive(forward, correct);
    }
    //*/

    /*// ------------- B O U N C E ---------------
      if(time < 0.6) {
        // D1
        drivetrain.arcadeDrive(forward, correct);
      } else if (time < 1.0) {
        // G1
        drIvetrain.arcadeDrive(0, -turn);
      } else if( time < 2.3) {
        // D2
        drivetrain.arcadeDrive(forward, correct);
      } else if( time < 3.6) {
        // D3
        drivetrain.arcadeDrive(-forward, correct);
      } else if (time < 4.0) {
        // G2
        drIvetrain.arcadeDrive(0,turn);
      } else if( time < 4.6) {
        // D4
        drivetrain.arcadeDrive(forward, correct);
      } else if (time < 5.0) {
        // G3
        drIvetrain.arcadeDrive(0, turn);
      } else if( time < 5.6) {
        // D5
        drivetrain.arcadeDrive(forward, correct);
      } else if (time < 6.0) {
        // G4
        drIvetrain.arcadeDrive(0, -turn);
      } else if( time < 6.6) {
        // D6
        drivetrain.arcadeDrive(forward, correct);
      } else if (time < 7.0) {
        // G5
        drIvetrain.arcadeDrive(0, -turn);
      } else if( time < 8.3) {
        // D7
        drivetrain.arcadeDrive(forward, correct);
      } else if( time < 9.6) {
        // D8
        drivetrain.arcadeDrive(-forward, correct);
      } else if (time < 10.0) {
        // G6
        drIvetrain.arcadeDrive(0, turn);
      } else if( time < 11.0) {
        // D9
        drivetrain.arcadeDrive(forward, correct);
      } else if (time < 11.4) {
        // G7
        drIvetrain.arcadeDrive(0, -turn);
      } else if( time < 12.7) {
        // D10
        drivetrain.arcadeDrive(forward, correct);
      } else if( time < 13.0) {
        // D11
        drivetrain.arcadeDrive(forward, correct);
      } else if (time < 13.4) {
        // G8
        drIvetrain.arcadeDrive(0, turn);
      } else if( time < 14) {
        // D12
        drivetrain.arcadeDrive(forward, correct);
      }
      //*/

    /*// ------------- B A R R E L ---------------
      if(time < 1.6) {
        // D1
        drivetrain.arcadeDrive(forward, correct);
      } else if (time < 2.0) {
        // G1
        drivetrain.arcadeDrive(0, turn);
      } else if( time < 2.6) {
        // D2
        drivetrain.arcadeDrive(forward, correct);
      } else if (time < 3.0) {
        // G2
        drivetrain.arcadeDrive(0, turn);
      } else if( time < 3.6) {
        // D3
        drivetrain.arcadeDrive(forward, correct);
      } else if (time < 4.0) {
        // G3
        drivetrain.arcadeDrive(0, turn);
      } else if (time < 4.6) {
        // D4
        drivetrain.arcadeDrive(forward, correct);
      } else if (time < 4.4) {
        // G4
        drivetrain.arcadeDrive(0, turn);
      } else if( time < 6.0) {
        // D5
        drivetrain.arcadeDrive(forward, correct);
      } else if (time < 6.4) {
        // G5
        drivetrain.arcadeDrive(0, turn);
      } else if( time < 7.0) {
        // D6
        drivetrain.arcadeDrive(forward, correct);
      } else if (time < 7.4) {
        // G6
        drivetrain.arcadeDrive(0, -turn);
      } else if (time < 8.0) {
        // D7
        drivetrain.arcadeDrive(forward, correct);
      } else if (time < 8.4) {
        // G7
        drivetrain.arcadeDrive(0, -turn);
      } else if( time < 9.6) {
        // D8
        drivetrain.arcadeDrive(forward, correct);
      } else if (time < 10.0) {
        // G8
        drivetrain.arcadeDrive(0, -turn);
      } else if( time < 11.6) {
        // D9
        drivetrain.arcadeDrive(forward, correct);
      } else if (time < 12.0) {
        // G9
        drivetrain.arcadeDrive(0, -turn);
      } else if (time < 12.6) {
        // D10
        drivetrain.arcadeDrive(forward, correct);
      } else if (time < 13.0) {
        // G10
        drivetrain.arcadeDrive(0, -turn);
      } else if (time < 15.0) {
        // D11
        drivetrain.arcadeDrive(forward, correct);
      }
    //*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
