/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.libs.auto.drive;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.libs.sensors.NavX;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Turn extends PIDCommand {
  /**
   * Creates a new Turn.
   */
  private static boolean killed = false;

  private DriveTrain m_dt;
  private ShuffleboardTab shooterTab;
  private NetworkTableEntry angleEntry;
  public Turn(DriveTrain dt, double setpoint, NavX navX) {
    super(
        // The controller that the command will use
        new PIDController(Constants.kP, Constants.kI, Constants.kD),
        // This should return the measurement
        () -> {
          return navX.getAngle();
        },
        // This should return the setpoint (can also be a constant)
        () -> {
          return setpoint;
        },
        // This uses the output
        output -> {
          if(output!=0.0){
            dt.arcadeDrive(0, output);
          }else{
            killed = true;
          }
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
    this.m_dt = dt;
    shooterTab = Shuffleboard.getTab("Shooter");
    angleEntry = shooterTab.add("Erro PID", 0).getEntry();
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return killed;
  }

  @Override
  public void execute() {
    angleEntry.setDouble(getController().getPositionError());
  }
}
