/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import javax.annotation.Nonnull;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tracker;

public class TurnLauncher extends CommandBase {

  // VARIABLES
  private boolean verticalTurn;
  private double degrees;
  private double degrees_original;

  // SUBSYSTEM
  private Tracker m_Tracker;

  /**
   * Creates a new TurnLauncher.
   * 
   * @param verticalTurn true if the turn is vertical
   * @param degrees      the degree of the turn, negative degrees works equal to
   *                     the cartesian plane.
   */
  public TurnLauncher(@Nonnull Tracker Tracker, @Nonnull boolean verticalTurn,@Nonnull double degrees) {
    m_Tracker = Tracker;
    this.verticalTurn = verticalTurn;
    this.degrees = degrees;
    addRequirements(m_Tracker);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    degrees_original = 0;
    //degrees = degrees_original + degrees - now;
    if(degrees < 0) {
      if(verticalTurn) {
        m_Tracker.turnVertical(false);
      } else {
        m_Tracker.turnHorizontal(false);
      }
    } else {
      if(verticalTurn) {
        m_Tracker.turnVertical(true);
      } else {
        m_Tracker.turnHorizontal(true);
      }
    
    }
  }

  private boolean proportional(double error) {
    return ((degrees_original /*- now*/) / 100 < error);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return proportional(0.1);
  }
}
