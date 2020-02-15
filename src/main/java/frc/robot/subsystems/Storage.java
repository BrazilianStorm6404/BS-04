/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.libs.sensors.Optical_RightSight;
import frc.robot.libs.sensors.Ultrasonic;

public class Storage extends SubsystemBase {

  private WPI_VictorSPX victor_storage;
  private Optical_RightSight limit_control;
  private Ultrasonic ultrasonic;

  /**
   * Creates a new Storage.
   */
  public Storage() {
    victor_storage = new WPI_VictorSPX(Constants.Motors.STORAGE.getPortCAN());
    limit_control = new Optical_RightSight(Constants.Sensors.LIMIT_IR_PRES.getPort());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean isCorrect() {
    return (limit_control.get() && ultrasonic.getRangeMM() < 450);
  }

  public void move(double force) {
    victor_storage.set(force);
  }
}
