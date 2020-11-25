/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libs.sensorsIMPL.Color_REV_V3;

public class Climb extends SubsystemBase {

  //#region Sensors, controllers & etc.
  private WPI_VictorSPX frontClimb, backClimb, telescopic;
  private SpeedControllerGroup climb;
  private Color_REV_V3 colorSensor;
  private NetworkTableEntry entryColorSensor;
  private ShuffleboardTab tabClimb;
  //#endregion

  public Climb() {
    // Controllers
    telescopic = new WPI_VictorSPX(Constants.Ports.Motors.CLIMB_TELESCOPIC);
    frontClimb = new WPI_VictorSPX(Constants.Ports.Motors.CLIMB_FRONT);
    backClimb = new WPI_VictorSPX(Constants.Ports.Motors.CLIMB_BACK);

    // Motor group
    climb = new SpeedControllerGroup(frontClimb, backClimb);

    // Sensor
    colorSensor = new Color_REV_V3(I2C.Port.kOnboard);

    // Shuffleboard
    tabClimb = Shuffleboard.getTab("Climb");
		entryColorSensor = tabClimb.add("Sensor de Cor", "Amarelo").getEntry();
  }

  @Override
  public void periodic() {
    // verificação se deve parar o telescopico de acordo com a cor
    if (colorSensor.getActualColorName() == "Yellow") {
      stopTelescopic();
    }

    // adiciona dado a shuffleboard
    entryColorSensor.forceSetString(colorSensor.getActualColorName());
  }

  //#region TELESCOPIC

  /**
   * Sets the telescopic speed
   * @param speed speed to be set
   */
  public void setTelescopic(double speed) {
    telescopic.set(speed);
  }

  /**
   * Sets the telescopic to the constant defined in Constants
   */
  public void raiseTelescopic(){
    telescopic.set(Constants.TELESCOPIC_SPEED_RAISE);
  }

  /**
   * Sets the telescopic to the constant defined in Constants in negative
   */
  public void lowerTelescopic(){
    telescopic.set(-Constants.TELESCOPIC_SPEED_LOWER);
  }

  /**
   * Sets the telescopic to 0
   */
  public void stopTelescopic() {
    telescopic.set(0.0);
  }
  //#endregion

  //#region CLIMBER
  /**
   * Sets the climber to the constant defined in Constants
   */
  public void climb (){
    climb.set(Constants.CLIMB_SPEED);
  }

  /**
   * Sets the climber to the constant defined in Constants in negative
   */
  public void inverseClimb (){
    climb.set(-Constants.CLIMB_SPEED);
  }

  /**
   * Sets the telescopic to 0
   */
  public void stopClimb (){
    climb.set(0);
  }
  //#endregion

}
