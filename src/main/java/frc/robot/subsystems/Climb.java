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
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libs.sensorsIMPL.Color_REV_V3;

public class Climb extends SubsystemBase {

  private WPI_VictorSPX frontClimb, backClimb, telescopic;
  private SpeedControllerGroup climb;
  private Color_REV_V3 colorSensor;
  private NetworkTableEntry entryColorSensor;
  private ShuffleboardTab tabClimb;

  public Climb() {
    telescopic = new WPI_VictorSPX(Constants.Ports.Motors.CLIMB_TELESCOPIC);
    frontClimb = new WPI_VictorSPX(Constants.Ports.Motors.CLIMB_FRONT);
    backClimb = new WPI_VictorSPX(Constants.Ports.Motors.CLIMB_BACK);
    climb = new SpeedControllerGroup(frontClimb, backClimb);
    colorSensor = new Color_REV_V3(I2C.Port.kOnboard);

    tabClimb = Shuffleboard.getTab("Climb");
		entryColorSensor = tabClimb.add("Sensor de Cor", "Amarelo").getEntry();
  }

  @Override
  public void periodic() {
    if (colorSensor.getActualColorName() == "Yellow") {
      stopTelescopic();
    }

    entryColorSensor.forceSetString(colorSensor.getActualColorName());
  }

  public void setTelescopic(double speed) {
    telescopic.set(speed);
  }
  
  public void raiseTelescopic(){
    telescopic.set(Constants.TELESCOPIC_SPEED_RAISE);
  }

  public void lowerTelescopic(){
    telescopic.set(-Constants.TELESCOPIC_SPEED_LOWER);
  }

  public void stopTelescopic() {
    telescopic.set(0.0);
  }
  
  public void climb (){
    climb.set(Constants.CLIMB_SPEED);
  }  
  
  public void stopClimb (){
    climb.set(0);
  }
  

}
