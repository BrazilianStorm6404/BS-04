/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private Spark angle;
  private VictorSP belt;
  private VictorSPX shoot;

  private ADXRS450_Gyro gyro;
  private DigitalInput limitHigh;

  public Shooter() {
    angle = new Spark(Constants.Ports.Motors.SHOOTER_ANGLE);
    belt = new VictorSP(Constants.Ports.Motors.SHOOTER_BELT);
    shoot = new VictorSPX(Constants.Ports.Motors.SHOOTER_SHOOT);

    gyro = new ADXRS450_Gyro();
    limitHigh = new DigitalInput(Constants.Ports.Sensors.SHOOTER_LIMIT_HIGH);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Shoot(){
  }

}
