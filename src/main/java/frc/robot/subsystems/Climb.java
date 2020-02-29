/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  
  private Spark leftClimb, rightClimb, telescopic;
  private SpeedControllerGroup climb;

  public Climb() {
    telescopic = new Spark(Constants.Ports.Motors.CLIMB_TELESCOPIC);

    leftClimb = new Spark(Constants.Ports.Motors.CLIMB_FRONT);
    rightClimb = new Spark(Constants.Ports.Motors.CLIMB_BACK);
    climb = new SpeedControllerGroup(leftClimb, rightClimb);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void runTelescopic(double speed){
    telescopic.set(speed);
  }
  
  public void climb (){
    climb.set(Constants.CLIMB_SPEED);
  }  
  
  public void stopClimb (){
    climb.set(0);
  }
  

}
