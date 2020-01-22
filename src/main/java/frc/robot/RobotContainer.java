/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import javax.annotation.Nonnegative;
import javax.annotation.Nonnull;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.libs.can.CANHelper;
import frc.robot.libs.auto.drive.*;
import frc.robot.libs.sensors.NavX;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Tracker;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //#region INSTANTIATION
  // CAN
  private CANHelper CAN = new CANHelper("1F6404FF");

  // CONTROLLERS
  private XboxController Controller1,Controller2;

  // BUTTONS CONTROLLER 1
  private JoystickButton ButtonA_1, ButtonB_1,ButtonX_1,ButtonY_1;

  // BUTTONS CONTROLLER 2
  private JoystickButton ButtonA_2,ButtonB_2,ButtonX_2,ButtonY_2;

  // MOTORS
  private WPI_VictorSPX LauncherPC,Storage,intakeMotor,StorageWheel,climbLeft,climbRight,telescopic;

  //SENSORS
  private NavX m_navx;

  // SUBSYSTEMS
  private final DriveTrain m_DriveTrain = new DriveTrain();
  private final Tracker m_Tracker = new Tracker();

  // COMMANDS

  private Straigth straigth = new Straigth(m_navx, m_DriveTrain, 0.0);
  
  //#endregion

  //#region CONSTRUCTOR
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    setControllers(1);
    configureButtonBindings();
    init();
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
    // LAUNCHER
    ButtonA_1.whenPressed(() -> LauncherPC.set(1.0))
    .whenReleased(() -> LauncherPC.set(0.0));
     
    // CLIMB
    ButtonB_1.whenPressed(() -> {
      climbLeft.set(Constants.Motors.getClimbSpeed());
      climbRight.set(Constants.Motors.getClimbSpeed());
    }).whenReleased(() -> {
      climbLeft.set(0.0);
      climbRight.set(0.0);
    });

    // TELECOSPIC
    ButtonX_1.whenPressed(() -> telescopic.set(Constants.Motors.getTelescopicSpeed())
    ).whenReleased(() -> telescopic.set(0.0));
    
    // INTAKE
    ButtonY_1.whenPressed(() -> intakeMotor.set(Constants.Motors.getIntakeSpeed()))
    .whenReleased(() -> intakeMotor.set(0.0));

    // EXECUTE EVERY PULSE
    CommandScheduler.getInstance().onCommandExecute(command -> { //can.get().toString();
      Shuffleboard.getTab("Logger").add("Log", "");
      m_DriveTrain.arcadeDrive(Controller1.getX(), Controller1.getY());

      if(CAN.readData("1F6404AA")[0] == (byte) 1) {
        new Turn(m_DriveTrain, 
        Shuffleboard.getTab("Vision").add("Angle", 0.0).getEntry().getDouble(0.0) + m_navx.getYaw(), m_navx);
      }
    });
  }
  //#endregion

  //#region SET CONTROLLERS 
  /**
   * Constructor for the OI
   * 
   * @param Qnt_Controllers quantity of controllers used in the robot.
   *                        <p>
   *                        <b><h10> MAXIMUM OF 2 CONTROLLERS </h10> </b>
   */
  public void setControllers(@Nonnull @Nonnegative int Qnt_Controllers) {
    // CONTROLLER 1
    Controller1 = new XboxController(Constants.OI_Map.CONTROLLER_1.getPort());
    ButtonA_1 = new JoystickButton(Controller1, Constants.OI_Map.BUTTON_A.getPort());
    ButtonB_1 = new JoystickButton(Controller1, Constants.OI_Map.BUTTON_B.getPort());
    ButtonX_1 = new JoystickButton(Controller1, Constants.OI_Map.BUTTON_X.getPort());
    ButtonY_1 = new JoystickButton(Controller1, Constants.OI_Map.BUTTON_Y.getPort());

    // CONTROLLER 2
    if (Qnt_Controllers == 2) {
      Controller2 = new XboxController(Constants.OI_Map.CONTROLLER_2.getPort());
      ButtonA_2 = new JoystickButton(Controller2, Constants.OI_Map.BUTTON_A.getPort());
      ButtonB_2 = new JoystickButton(Controller2, Constants.OI_Map.BUTTON_B.getPort());
      ButtonX_2 = new JoystickButton(Controller2, Constants.OI_Map.BUTTON_X.getPort());
      ButtonY_2 = new JoystickButton(Controller2, Constants.OI_Map.BUTTON_Y.getPort());
    }
  }
  //#endregion

  //#region INITIALIZER
  /**
   * Initializer of almost every item in the robot.
   */
  public void init() {
    // MOTORS
    climbLeft = new WPI_VictorSPX(Constants.Motors.CLIMB_LEFT.getPort());
    climbRight = new WPI_VictorSPX(Constants.Motors.CLIMB_RIGHT.getPort());
    telescopic = new WPI_VictorSPX(Constants.Motors.TELESCOPIC.getPort());
    intakeMotor = new WPI_VictorSPX(Constants.Motors.INTAKE.getPort());
    LauncherPC = new WPI_VictorSPX(Constants.Motors.SHOOTER.getPort());
    Storage = new WPI_VictorSPX(Constants.Motors.STORAGE.getPort());
    StorageWheel = new WPI_VictorSPX(Constants.Motors.STORAGE_WHEEL.getPort());

    // SENSORS
    m_navx = new NavX(SerialPort.Port.kMXP);
  }
  //#endregion

  //#region AUTONOMOUS
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return () -> {
      m_Tracker.shoot();
      return null;
    };
  }
  //#endregion
}
