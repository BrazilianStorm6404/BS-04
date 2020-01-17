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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // SUBSYSTEMS


  // CONTROLLERS
  private XboxController Controller1;
  private XboxController Controller2;

  // BUTTONS CONTROLLER 1
  private JoystickButton ButtonA_1;
  private JoystickButton ButtonB_1;
  private JoystickButton ButtonX_1;
  private JoystickButton ButtonY_1;

  // BUTTONS CONTROLLER 2
  private JoystickButton ButtonA_2;
  private JoystickButton ButtonB_2;
  private JoystickButton ButtonX_2;
  private JoystickButton ButtonY_2;

  // MOTORS
  WPI_VictorSPX LauncherPC;
  WPI_VictorSPX Storage;
  WPI_VictorSPX StorageWheel;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    setControllers(1);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    ButtonA_1.whenPressed(() -> LauncherPC.set(1.0))
    .whenReleased(() -> LauncherPC.set(0.0));
    
    //Logger
    CommandScheduler.getInstance().onCommandExecute(command -> { //can.get().toString();
      Shuffleboard.getTab("Logger").add("Log", "");
    });
  }

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

  /**
   * Creates a new Launcher with it's sensors.
   */
  public void LauncherInit() {
    // MOTORS
    LauncherPC = new WPI_VictorSPX(Constants.Motors.LAUNCHER_PC.getPort());
  }

  /**
   * Creates a new Storage with it's sensors.
   */
  public void StorageInit() {
    Storage = new WPI_VictorSPX(Constants.Motors.STORAGE.getPort());
    StorageWheel = new WPI_VictorSPX(Constants.Motors.STORAGE_WHEEL.getPort());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
