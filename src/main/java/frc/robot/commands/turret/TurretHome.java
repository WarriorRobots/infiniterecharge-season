/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vars;
import frc.robot.subsystems.TurretSubsystem;

public class TurretHome extends CommandBase {
  TurretSubsystem m_clank;
  private double target;

  /**
   * Rotates the target to the specified target rotation.
   * 
   * @param target The target the turret is to rotate to in degrees.
   */
  public TurretHome(TurretSubsystem clank, double target) {
    m_clank = clank;
    addRequirements(this.m_clank);
    this.target = target;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_clank.rotateToPosition(target);
  }
  
  @Override
  public boolean isFinished() {
    return Math.abs(target-m_clank.getRotationDegrees())<Vars.TURRET_TOLERANCE;
  }
  
  @Override
  public void end(boolean interrupted) {
    m_clank.stop();
  }
}