// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.DataStructure;
import frc.robot.subsystems.Drivetrain;

public class resetCommand extends CommandBase {

  private Drivetrain drivetrain;
  private boolean finished = false, angled = false;
  File f = new File("/home/lvuser/Logs/joystick.txt");
  /*private DataStructure[] datalist = { new DataStructure(false, 1.0, -0.9, 0.4), // D1
      new DataStructure(true, -90.0, 0.0, -0.8), // G1
      new DataStructure(false, 1.0, -0.9, 0.4), // D2
      new DataStructure(true, 90.0, 0.0, 0.8), // G2
      new DataStructure(false, 2.7, -0.9, 0.4), // D3
      new DataStructure(true, 90.0, 0.0, 0.8), // G3
      new DataStructure(false, 1.0, -0.9, 0.4), // D4
      new DataStructure(true, -90.0, 0.0, -0.8), // G4
      new DataStructure(false, 1.0, -0.9, 0.4), // D5
      new DataStructure(true, -90.0, 0.0, -0.8), // G5
      new DataStructure(false, 1.0, -0.9, 0.4), // D6
      new DataStructure(true, -90.0, 0.0, -0.8), // G6
      new DataStructure(false, 1.0, -0.9, 0.4), // D7
      new DataStructure(true, -90.0, 0.0, -0.8), // G7
      new DataStructure(false, 1.0, -0.9, 0.4), // D8
      new DataStructure(true, 90.0, 0.0, 0.8), // G8
      new DataStructure(false, 2.7, -0.9, 0.4), // D9
      new DataStructure(true, 90.0, 0.0, 0.8), // G9
      new DataStructure(false, 1.0, -0.9, 0.4), // D10
      new DataStructure(true, -90.0, 0.0, -0.8), // G10
      new DataStructure(false, 1.0, -0.9, 0.4) // D11
  };*/
  Timer timer = new Timer();
  double angle = 0;
  Scanner myReader;

  /** Creates a new resetCommand. */
  public resetCommand(Drivetrain m_Drivetrain) {
    addRequirements(m_Drivetrain);
    drivetrain = m_Drivetrain;
    try {
      myReader = new Scanner(f);
    } catch (FileNotFoundException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
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
    //double forward = -0.6, turn = 0.6, correct = 0, time = timer.get();
    // 90 graus 0.5
    // ~1.73m/s
    // incrementos de 1m
      if (f.exists() && f.canRead()) {
          if(myReader.hasNextLine()) {
            String data = myReader.nextLine();
            String[] array = data.split("i");
            SmartDashboard.putString("key", data);
            drivetrain.arcadeDrive(Double.parseDouble(array[0]), Double.parseDouble(array[1]));
          } else {
            myReader.close();
            finished = true;
          }
      }
    /*
     * if(timer.get() > 0.5) { for(int i = 0; i < datalist.length; i++) {
     * while(datalist[i].valor > Math.abs(datalist[i].girar ?
     * drivetrain.getHeading() : drivetrain.getEncoderLeft())) {
     * drivetrain.arcadeDrive(datalist[i].forward, datalist[i].rot); }
     * drivetrain.resetEncoders(); drivetrain.zeroHeading(); } }
     */  
    
    /*// ------------- S H A L O M ---------------
    if (time < 1.6) {
      // D1
      drivetrain.arcadeDrive(forward, correct);
    } else if (time < 3.5) {
      // G1
      drivetrain.arcadeDrive(0, -turn);
    } else if (time < 4.9) {
      // D2
      angled = false;
       drivetrain.arcadeDrive(forward, correct);
    } else if (time < 6.6) {
      // G2
      drivetrain.arcadeDrive(0, turn);
    } else if (time < 9.3) {
      // D3
      angled = false;
      drivetrain.arcadeDrive(forward, correct-correct);
    } /*else if (time < 4.3 ) {
      // G3
      if(!angled) {
        angle = drivetrain.getHeading();
      }
      angled = true;
      drivetrain.arcadeDrive(0, turn);
    } else if (time < 4.9) {
      // D4
      angled = false;
      drivetrain.arcadeDrive(forward, correct);
    } else if (time < 5.3) {
      // G4
      drivetrain.arcadeDrive(0, -turn);
    } else if (time < 6.0) {
      // D5
      angled = false;
      drivetrain.arcadeDrive(forward, correct);
    } else if (time < 6.5) {
      // G5
      angled = true;
      drivetrain.arcadeDrive(0, -turn);
    } else if (time < 7.1) {
      // D6
      angled = false;
      drivetrain.arcadeDrive(forward, correct);
    } else if (time < 7.5) {
      // G6
      drivetrain.arcadeDrive(0, -turn);
    } else if (time < 8.0) {
      // D7
      drivetrain.arcadeDrive(forward, correct);
    } else if (time < 8.4) {
      // G7

      angled = true;
      drivetrain.arcadeDrive(0, -turn);
    } else if (time < 9.0) {
      // D8
      angled = false;
      drivetrain.arcadeDrive(forward, correct);
    } else if (time < 9.4) {
      // G8
      drivetrain.arcadeDrive(0, turn);
    } else if( time < 11.0) {
      // D9
      angled = false;
      drivetrain.arcadeDrive(forward, correct);
    } else if (time < 11.4 ) {
      // G9
      drivetrain.arcadeDrive(0, turn);
    } else if (time < 12.0) {
      // D10
      angled = false;
      drivetrain.arcadeDrive(forward, correct);
    } else if (time < 12.4) {
      // G10
      drivetrain.arcadeDrive(0, -turn);
    } else if (time < 13.0) {
      // D11
      drivetrain.arcadeDrive(forward, correct);
    } else {
      finished = true;
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
