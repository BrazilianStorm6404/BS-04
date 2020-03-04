/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ActivateShooter;
import frc.robot.commands.ControlStorage;
import frc.robot.commands.Shoot;
import frc.robot.libs.auto.drive.Straight;
import frc.robot.libs.sensorsIMPL.Pixy;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;

  
public class RobotContainer {

  private XboxController pilot, COpilot;

  // BUTTONS CONTROLLER 1
  private JoystickButton pilot_RB, pilot_ButtonA;

  // BUTTONS CONTROLLER 2
  private JoystickButton CO_ButtonX, CO_ButtonB, CO_ButtonY;

  // SENSORS
  private AHRS m_navx;
  private Pixy m_pixy;

  //This must be refactored
  //private Encoder_AMT103 encoderT1;

  // SUBSYSTEMS
  private final Climb m_Climb = new Climb();
  private Drivetrain m_DriveTrain;
  private final Shooter m_Shooter = new Shooter();
  private final Storage m_Storage = new Storage();
  private PowerDistributionPanel m_PDP;

  // COMMANDS

  // SHUFFLEBOARD


  //#region CONSTRUCTOR
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    setControllers();
    configureButtonBindings();
    init();
  }
  //#endregion


  public void setControllers() {

    pilot = new XboxController(Constants.OI_Map.PILOT);
    pilot_RB = new JoystickButton(pilot, Constants.OI_Map.BUTTON_RIGHT);
    pilot_ButtonA = new JoystickButton(pilot, Constants.OI_Map.BUTTON_A);

    COpilot = new XboxController(Constants.OI_Map.COPILOT);
    CO_ButtonX = new JoystickButton(COpilot, Constants.OI_Map.BUTTON_X);
    CO_ButtonB = new JoystickButton(COpilot, Constants.OI_Map.BUTTON_B);
    CO_ButtonY = new JoystickButton(COpilot, Constants.OI_Map.BUTTON_Y);
    }

  //#region BUTTON BINDINGS
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    CO_ButtonX.whenPressed(()-> m_Storage.pullPowerCell(), m_Storage)
      .whenReleased(() -> m_Storage.stopPowerCell(), m_Storage);
    
      new JoystickButton(pilot, Constants.OI_Map.BUTTON_X).whenPressed(() -> m_Storage.reversePowerCell(),m_Storage)
      .whenReleased(() -> m_Storage.stopPowerCell(), m_Storage);
    pilot_ButtonA.whileHeld(() -> m_DriveTrain.arcadeDrive(0, 1), m_DriveTrain);

    CO_ButtonB.whileHeld(()-> {
      if(-COpilot.getY(GenericHID.Hand.kLeft) > 0.2){
        m_Climb.climb();
      }
      else m_Climb.stopClimb();
    }, m_Climb)
    .whenReleased(()-> m_Climb.stopClimb(), m_Climb);

    CO_ButtonY.whileHeld(() -> {
      m_Storage.MoveBelt();
    }, m_Storage).whenReleased(() -> m_Storage.stopBelt(), m_Storage);

    pilot_RB.whileHeld(new Shoot(m_Shooter, m_Storage, m_PDP))
    .whenReleased(new RunCommand(()->{

      m_Shooter.stopShooting();
      m_Shooter.stopBelt();
      m_Storage.MoveBelt();

    }, m_Shooter, m_Storage));

  }
  //#endregion

  public void init() {

    // SENSORS
    m_navx = new AHRS(SerialPort.Port.kUSB);
    m_DriveTrain = new Drivetrain(m_navx);
    m_PDP  = new PowerDistributionPanel(0);
  }
  
  public Command getAutonomousCommand() {
    m_navx.reset();
    // Implementar rotina autônoma.
    //*** */
    // An ExampleCommand will run in autonomous
    var autoVoltageConstraint = 
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.Autonomous.ksVolts, 
        Constants.Autonomous.kvVoltSecondsPerMeter, Constants.Autonomous.kaVoltSecondsSquaredPerMeter),
        Constants.Autonomous.kDriveKinematics,
        10);
      
    TrajectoryConfig config =
        new TrajectoryConfig(Constants.Autonomous.kMaxSpeedMetersPerSecond,
                            Constants.Autonomous.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.Autonomous.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

                // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(
          new Translation2d(1, 1),
          new Translation2d(2, -1)
      ),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(3, 0, new Rotation2d(0)),
      // Pass config
      config
  );

  RamseteCommand ramseteCommand = new RamseteCommand(
      exampleTrajectory,
      m_DriveTrain::getPose,
      new RamseteController(Constants.Autonomous.kRamseteB, Constants.Autonomous.kRamseteZeta),
      new SimpleMotorFeedforward(Constants.Autonomous.ksVolts,
                                Constants.Autonomous.kvVoltSecondsPerMeter,
                                Constants.Autonomous.kaVoltSecondsSquaredPerMeter),
      Constants.Autonomous.kDriveKinematics,
      m_DriveTrain::getWheelSpeeds,
      new PIDController(Constants.Autonomous.kPDriveVel, 0, 0),
      new PIDController(Constants.Autonomous.kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      m_DriveTrain::tankDriveVolts,
      m_DriveTrain
  );

  // Run path following command, then stop at the end.
  return ramseteCommand.andThen(() -> m_DriveTrain.tankDriveVolts(0, 0));
    /*
    return new SequentialCommandGroup(
      new Straight(m_navx, m_DriveTrain, 0, 0.3)
    );
    */
  }


  public void callBinders() {

    m_DriveTrain.setDefaultCommand(new RunCommand(() -> {

      m_DriveTrain.arcadeDrive(pilot.getY(GenericHID.Hand.kLeft), pilot.getX(GenericHID.Hand.kRight));

    }, m_DriveTrain));

    m_Climb.setDefaultCommand(new RunCommand(()->{

      double delta = COpilot.getTriggerAxis(Hand.kRight) - COpilot.getTriggerAxis(Hand.kLeft);
      if (delta > 0.1) {
        m_Climb.setTelescopic(delta * Constants.TELESCOPIC_SPEED_RAISE);  
      } else {
        m_Climb.setTelescopic(delta * Constants.TELESCOPIC_SPEED_LOWER);
      }

    }, m_Climb));

    m_Shooter.setDefaultCommand(new RunCommand(()->{

      final double delta = pilot.getTriggerAxis(Hand.kRight) - pilot.getTriggerAxis(Hand.kLeft);

      if(delta > 0.1) m_Shooter.moveUp();
      else if (delta < -0.1) m_Shooter.moveDown();
      else m_Shooter.stopMoving();

    }, m_Shooter));

    m_Storage.setDefaultCommand(new ControlStorage(m_Storage));
  }
}
