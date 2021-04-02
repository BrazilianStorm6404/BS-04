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
import frc.robot.libs.sensorsIMPL.Pixy;
import frc.robot.subsystems.Drivetrain;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

public class resetCommand extends CommandBase {

  private Drivetrain drivetrain;
  private boolean finished = false;
  File f = new File("/home/lvuser/Logs/joystick.txt"),
  f2 = new File("/home/lvuser/Deploy/route1.txt"),
  f3 = new File("/home/lvuser/Deploy/route2.txt");
  double angle = 0;
  Scanner myReader;
  Pixy pixy;

  /** Creates a new resetCommand. */
  public resetCommand(Drivetrain m_Drivetrain,Pixy _pixy) {
    addRequirements(m_Drivetrain);
    drivetrain = m_Drivetrain;
    pixy = _pixy;
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double forward = -0.6, turn = 0.6, correct = 0, time = timer.get();
    // 90 graus 0.5
    // ~1.73m/s
    // incrementos de 1m
    Block b = pixy.getBiggestBlock();
      if(b != null) {
        if (f2.exists() && f2.canRead()) {
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
     } else {  
      SmartDashboard.putNumber("key", 0);
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
     }
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
