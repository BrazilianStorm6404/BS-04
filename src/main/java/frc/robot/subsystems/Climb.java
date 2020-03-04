/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  
  private WPI_VictorSPX leftClimb, rightClimb;
  private Spark telescopic;
  private SpeedControllerGroup climb;

  public Climb() {
    telescopic = new Spark(Constants.Ports.Motors.CLIMB_TELESCOPIC);
    leftClimb = new WPI_VictorSPX(Constants.Ports.Motors.CLIMB_LEFT);
    rightClimb = new WPI_VictorSPX(Constants.Ports.Motors.CLIMB_RIGHT);
    climb = new SpeedControllerGroup(leftClimb, rightClimb);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setTelescopic(double speed) {
    telescopic.set(speed);
  }
  
  public void raiseTelescopic(){
    telescopic.set(Constants.TELESCOPIC_SPEED_RAISE);
  }

  public void lowerTelescopic(){
    telescopic.set(-Constants.TELESCOPIC_SPEED_LOWER);
  }

  public void stopTelescopic() {
    telescopic.set(0.0);
  }
  
  public void climb (){
    climb.set(Constants.CLIMB_SPEED);
  }  
  
  public void stopClimb (){
    climb.set(0);
  }
  

}
