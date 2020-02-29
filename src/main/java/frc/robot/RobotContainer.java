/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ControlStorage;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShooterAlign;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;

  
public class RobotContainer {

  //#region INSTANCES
  private XboxController pilot, COpilot;

  // BUTTONS CONTROLLER 1
  private JoystickButton pilot_RB, pilot_ButtonA;

  // BUTTONS CONTROLLER 2
  private JoystickButton CO_ButtonX, CO_ButtonB;

  // SENSORS
  private AHRS m_navx;

  // MOTORS
  private WPI_VictorSPX intake;

  // SUBSYSTEMS
  private Climb m_Climb;
  private Drivetrain m_DriveTrain;
  private Shooter m_Shooter;
  private Storage m_Storage;
  private PowerDistributionPanel m_PDP;

  // COMMANDS

  //#endregion

  //#region CONSTRUCTOR
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    init();
    setControllers();
    configureButtonBindings();
  }
  //#endregion

  //#region SET CONTROLLERS
  public void setControllers() {
    pilot = new XboxController(Constants.OI_Map.PILOT);
    pilot_RB = new JoystickButton(pilot, Constants.OI_Map.BUTTON_RIGHT);
    pilot_ButtonA = new JoystickButton(pilot, Constants.OI_Map.BUTTON_A);

    COpilot = new XboxController(Constants.OI_Map.COPILOT);
    CO_ButtonX = new JoystickButton(COpilot, Constants.OI_Map.BUTTON_X);
    CO_ButtonB = new JoystickButton(COpilot, Constants.OI_Map.BUTTON_B);
    }
  //#endregion

  //#region BUTTON BINDINGS
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // BUTTON X OF COPILOT = INTAKE
    CO_ButtonX.whenPressed(()-> intake.set(0.0), m_Storage)
      .whenReleased(() -> intake.set(Constants.INTAKE_SPEED), m_Storage);

    // BUTTON A OF PILOT = TURN
    pilot_ButtonA.whileHeld(() -> m_DriveTrain.arcadeDrive(0, 1), m_DriveTrain);

    // BUTTON B OF COPILOT = CLIMB
    CO_ButtonB.whileHeld(()-> {
      if(-COpilot.getY(GenericHID.Hand.kLeft) > 0.2){
        m_Climb.climb();
      }
      else m_Climb.stopClimb();
    }, m_Climb)
    .whenReleased(()-> m_Climb.stopClimb(), m_Climb);

    // BUTTON X OF COPILOT = SHOOTER
    pilot_RB.whenPressed(new SequentialCommandGroup(new ShooterAlign(m_Shooter), 
      new Shoot(m_Shooter, m_Storage, m_PDP)));

  }
  //#endregion 

  //#region INIT
  public void init() {
    // MOTORS
    intake = new WPI_VictorSPX(Constants.Ports.Motors.INTAKE_COLLECTOR);

    // SENSORS
    m_navx = new AHRS(SerialPort.Port.kMXP);
    m_PDP  = new PowerDistributionPanel(0);

    // SUBSYSTEMS
    m_Storage = new Storage();
    m_Shooter= new Shooter();
    m_Climb = new Climb();
    m_DriveTrain = new Drivetrain(m_navx);
  }
  //#endregion
  
  //#region AUTONOMOUS
  public Command getAutonomousCommand() {
    m_navx.reset();

    return new SequentialCommandGroup(
      new PIDCommand(new PIDController(Constants.drive_kP,Constants.drive_kI,Constants.drive_kD),
        () -> m_navx.getYaw(),
        0,
        output -> m_DriveTrain.arcadeDrive(0.0, output),
        m_DriveTrain)
    );
  }
  //#endregion

  //#region CALL BINDERS
  public void callBinders() {

    m_DriveTrain.setDefaultCommand(new RunCommand(() -> m_DriveTrain.arcadeDrive(-pilot.getY(GenericHID.Hand.kLeft), 
      pilot.getX(GenericHID.Hand.kRight)), m_DriveTrain));

    m_Climb.setDefaultCommand(new RunCommand(()->
      m_Climb.runTelescopic((COpilot.getTriggerAxis(Hand.kRight) 
      - COpilot.getTriggerAxis(Hand.kLeft))), m_Climb));

    m_Storage.setDefaultCommand(new ControlStorage(m_Storage));
  }
  //#endregion
}
