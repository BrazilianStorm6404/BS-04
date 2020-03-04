/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;

public class Shoot extends CommandBase {
  
  private Shooter m_shooter;
  private Storage m_storage;
  private PowerDistributionPanel m_pdp;

  public Shoot(Shooter Shooter, Storage Storage, PowerDistributionPanel PDP) {
    addRequirements(Shooter,Storage);
    m_shooter = Shooter;
    m_storage = Storage;
    m_pdp = PDP;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_storage.shoot = true;
    /*
    numBalls = 0;
    t.start();
    for (boolean ball : _storage.balls) 
      if (ball)
        numBalls++;
    */
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.Shoot();
    m_storage.MoveBelt(Constants.STORAGE_BELT_SPEED);

    // Ajustar constante de corrente da Power Distribution Panel.
    // Verificar porta da PDP.
    //***
    SmartDashboard.putNumber("motor", m_pdp.getCurrent(Constants.Ports.Motors.SHOOTER_SHOOT));
    if(m_pdp.getCurrent(Constants.Ports.Motors.SHOOTER_SHOOT)>10)
      m_storage.removePowerCells();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_storage.shoot = false;
    m_shooter.stopShooting();
    m_storage.MoveBelt(Constants.STORAGE_BELT_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_storage.getPowerCellCount() <= 0;
  }
}
