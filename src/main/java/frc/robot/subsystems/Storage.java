/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Storage extends SubsystemBase {

  private VictorSPX belt, joint, intake;


  private DigitalInput lowerLimit, IR_Internal;
  private Ultrasonic ultrasonic;


  public Storage() {
    belt = new VictorSPX(Constants.Ports.Motors.STORAGE_BELT);
    joint = new VictorSPX(Constants.Ports.Motors.COLLECTOR_JOINT);
    intake = new VictorSPX(Constants.Ports.Motors.COLLECTOR_INTAKE);

    lowerLimit = new DigitalInput(Constants.Ports.Sensors.COLLECTOR_LIMIT);
    IR_Internal = new DigitalInput(Constants.Ports.Sensors.STORAGE_IR);
    ultrasonic = new  Ultrasonic(Constants.Ports.Sensors.STORAGE_ULTRASONIC_PING, Constants.Ports.Sensors.STORAGE_ULTRASONIC_ECHO);;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
