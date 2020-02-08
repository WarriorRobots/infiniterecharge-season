/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.IO;
import frc.robot.subsystems.TurretSubsystem;

public class TurretRotate extends CommandBase {
  TurretSubsystem m_clank;

  /**
   * Rotate the turret linearly by use a supplier.
   * 
   * @param input A supplier/lambda that gives a double from a joystick or other input.
   */
  public TurretRotate(TurretSubsystem clank) {
    m_clank = clank;
    addRequirements(this.m_clank);
  }
  
  @Override
  public void initialize() {
  }
  
  @Override
  public void execute() {
    m_clank.rotate(IO.getRightX());
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
  
  @Override
  public void end(boolean interrupted) {
    m_clank.stop();
  }
}