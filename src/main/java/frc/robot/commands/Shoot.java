/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;

public class Shoot extends CommandBase {
  
  private Shooter _shooter;
  private Storage _storage;
  private PowerDistributionPanel _pdp;
  private int numBalls;
  private Timer t;

  public Shoot(Shooter m_Shooter, Storage m_Storage, PowerDistributionPanel m_PDP) {
    _shooter = m_Shooter;
    _storage = m_Storage;
    _pdp = m_PDP;
    t = new Timer();
    addRequirements(m_Shooter,m_Storage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
    _shooter.Shoot();
    _storage.MoveBelt(1);
    _shooter.moveBelt();

    // Ajustar constante de corrente da Power Distribution Panel.
    // Verificar porta da PDP.
    //***
    /*
    if(_pdp.getCurrent(Constants.Ports.Motors.SHOOTER_SHOOT)>10 && t.get() > 1){
      numBalls--;
      t.reset();
      t.start();
    }
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _shooter.stopShooting();
    _storage.MoveBelt(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return numBalls <= 0;
    return false;
  }
}
