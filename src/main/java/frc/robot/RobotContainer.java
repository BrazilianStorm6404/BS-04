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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.libs.auto.drive.Straigth;
import frc.robot.libs.can.CANHelper;
import frc.robot.libs.logger.Logger;
import frc.robot.libs.sensors.Encoder_AMT103;
import frc.robot.libs.sensors.Gyro_ADXRS450;
import frc.robot.libs.sensors.NavX;
import frc.robot.libs.sensors.Pixy;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Storage;
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
  // LOGGER
  Logger logger;

  // SHUFFLEBOARD

  // CAN
  //private CANHelper CAN = new CANHelper("1F6404FF");

  // CONTROLLERS
  private XboxController Controller1, Controller2;
  private Joystick joystick1;

  // BUTTONS CONTROLLER 1
  private JoystickButton ButtonA_1, ButtonB_1, ButtonX_1, ButtonY_1;

  // BUTTONS CONTROLLER 2
  private JoystickButton ButtonA_2, ButtonB_2, ButtonX_2, ButtonY_2;

  // MOTORS
  private WPI_VictorSPX  intakeMotor, climbLeft, climbRight, telescopic;

  // SENSORS
  private NavX m_navx;
  private Pixy m_pixy;
  private Encoder_AMT103 encoderT1;

  // SUBSYSTEMS
  private final DriveTrain m_DriveTrain = new DriveTrain();
  private final Tracker m_Tracker = new Tracker();
  private final Storage m_Storage = new Storage();

  // COMMANDS

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
    ButtonA_1.whenPressed(() -> 
      m_navx.reset()
    ).whenHeld(
      new PIDCommand(new PIDController(Constants.kP,Constants.kI,Constants.kD),
      () -> m_navx.getYaw(),
      90,
      output -> m_DriveTrain.arcadeDrive(-Controller1.getY(GenericHID.Hand.kLeft), output),
      m_DriveTrain)
    );

    // CLIMB
    ButtonB_1.whenPressed(() -> {
      climbLeft.set(Constants.Motors.getClimbSpeed());
      climbRight.set(Constants.Motors.getClimbSpeed());
    }).whenReleased(() -> {
      climbLeft.set(0.0);
      climbRight.set(0.0);
    });

    // TELESCOPIC
    ButtonX_1.whenPressed(() -> telescopic.set(Constants.Motors.getTelescopicSpeed()))
        .whenReleased(() -> telescopic.set(0.0));

    // INTAKE
    ButtonY_1.whenPressed(() -> intakeMotor.set(Constants.Motors.getIntakeSpeed()))
        .whenReleased(() -> intakeMotor.set(0.0));
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
    // JOYSTICK 1
    joystick1 = new Joystick(0);
  }
  //#endregion

  //#region INITIALIZER
  /**
   * Initializer of almost every item in the robot.
   */
  public void init() {
    // MOTORS
    climbLeft = new WPI_VictorSPX(Constants.Motors.CLIMB_LEFT.getPortCAN());
    climbRight = new WPI_VictorSPX(Constants.Motors.CLIMB_RIGHT.getPortCAN());
    telescopic = new WPI_VictorSPX(Constants.Motors.TELESCOPIC.getPortCAN());
    intakeMotor = new WPI_VictorSPX(Constants.Motors.INTAKE.getPortCAN());

    // SENSORS
    m_navx = new NavX(SerialPort.Port.kMXP);
    encoderT1 = new Encoder_AMT103(Constants.Sensors.ENC_T1_WHEEL_A.getPort(),Constants.Sensors.ENC_T1_WHEEL_B.getPort(),true);
    encoderT1.setDistancePerPulse(Math.PI * 4 * 2.54/ 360.0);
    encoderT1.setMinRate(1.0);
    encoderT1.setSamplesToAverage(5);
    m_pixy = new Pixy();
    m_pixy.initialize();

    // SHUFFLEBOARD

    // LOGGER
    logger = new Logger();
  }
  //#endregion

  //#region AUTONOMOUS
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    m_navx.reset();
    // An ExampleCommand will run in autonomous
    return new SequentialCommandGroup(
      new PIDCommand(new PIDController(Constants.kP,Constants.kI,Constants.kD),
        () -> m_navx.getYaw(),
        0,
        output -> m_DriveTrain.arcadeDrive(0.0, output),
        m_DriveTrain)
    );
  }
  //#endregion

  //#region CALL BINDERS
  public void callBinders() {
    // EXECUTE EVERY PULSE
    m_DriveTrain.setDefaultCommand(new RunCommand(() -> {
      /*
      Block b = m_pixy.getBiggestBlock();
      if (b != null) {
        double d = (b.getX() / 360.0), a = (b.getHeight() * b.getWidth()) / 10000.0;
        m_DriveTrain.arcadeDrive(1.0 - a, d < 0.5 ? (d > -0.5 ? -0.6 : d) : d);
        System.out.println(1.0 - a);
      } else {
      */
        m_DriveTrain.arcadeDrive(-Controller1.getY(GenericHID.Hand.kLeft), Controller1.getX(GenericHID.Hand.kRight));
        //m_DriveTrain.arcadeDrive(-joystick1.getY(), joystick1.getZ());
    }, m_DriveTrain));

    m_Storage.setDefaultCommand(new RunCommand(() -> {
      if(!m_Storage.isCorrect()) {
        m_Storage.move(-0.5);
      } else {
        m_Storage.move(0.0);
      }
    },m_Storage));
  }
  //#endregion
}
