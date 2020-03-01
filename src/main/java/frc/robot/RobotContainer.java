/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ActivateShooter;
import frc.robot.commands.ControlStorage;
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
  private JoystickButton CO_ButtonX, CO_ButtonB;

  // SENSORS
  private AHRS m_navx;
  private Pixy m_pixy;

  //This must be refactored
  //private Encoder_AMT103 encoderT1;

  // SUBSYSTEMS
  private final Climb m_Climb = new Climb();
  private final Drivetrain m_DriveTrain = new Drivetrain( m_navx);
  private final Shooter m_Shooter = new Shooter();
  private final Storage m_Storage = new Storage();
  private PowerDistributionPanel m_PDP;

  // COMMANDS


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

    pilot_ButtonA.whileHeld(() -> m_DriveTrain.arcadeDrive(0, 1), m_DriveTrain);

    CO_ButtonB.whileHeld(()-> {
      if(-COpilot.getY(GenericHID.Hand.kLeft) > 0.2){
        m_Climb.climb();
      }
      else m_Climb.stopClimb();
    }, m_Climb)
    .whenReleased(()-> m_Climb.stopClimb(), m_Climb);

    pilot_RB.whenPressed(new ActivateShooter(m_Shooter, m_Storage, m_PDP));

  }
  //#endregion

  public void init() {

    // SENSORS
    m_navx = new AHRS(SerialPort.Port.kMXP);
    
    m_PDP  = new PowerDistributionPanel(0);


  }
  
  public Command getAutonomousCommand() {
    m_navx.reset();
    // Implementar rotina autônoma.
    //*** */
    // An ExampleCommand will run in autonomous
    return new SequentialCommandGroup(
      new PIDCommand(new PIDController(Constants.DRIVE_kP,Constants.DRIVE_kI,Constants.DRIVE_kD),
        () -> m_navx.getYaw(),
        0,
        output -> m_DriveTrain.arcadeDrive(0.0, output),
        m_DriveTrain)
    );
  }


  public void callBinders() {

    m_DriveTrain.setDefaultCommand(new RunCommand(() -> {

      m_DriveTrain.arcadeDrive(-pilot.getY(GenericHID.Hand.kLeft), pilot.getX(GenericHID.Hand.kRight));

    }, m_DriveTrain));

    m_Climb.setDefaultCommand(new RunCommand(()->{

      final double delta = COpilot.getTriggerAxis(Hand.kRight) - COpilot.getTriggerAxis(Hand.kLeft);

      if(delta > 0.1) m_Climb.raiseTelescopic();
      else if (delta < -0.1) m_Climb.lowerTelescopic();
      else m_Climb.stopTelescopic();

    }, m_Climb));

    m_Storage.setDefaultCommand(new ControlStorage(m_Storage));
  }
}
