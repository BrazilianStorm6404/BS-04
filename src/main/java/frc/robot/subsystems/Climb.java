/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  WPI_VictorSPX climbLeft, climbRight, telescopic;
  /**
   * Creates a new Climb.
   */
  public Climb() {
    climbLeft = new WPI_VictorSPX(Constants.Climb.CLIMB_LEFT.getPort());
    climbRight = new WPI_VictorSPX(Constants.Climb.CLIMB_RIGHT.getPort());
    telescopic = new WPI_VictorSPX(Constants.Climb.TELESCOPIC.getPort());
  }

  public void climb() {
    climbLeft.set(Constants.Climb.getClimbSpeed());
    climbRight.set(Constants.Climb.getClimbSpeed());
  }

  public void stopClimb() {
    climbLeft.set(0.0);
    climbRight.set(0.0);
  }

  public void startTelescopic() {
    telescopic.set(Constants.Climb.getTelescopicSpeed());
  }

  public void stopTelescopic() {
    telescopic.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
