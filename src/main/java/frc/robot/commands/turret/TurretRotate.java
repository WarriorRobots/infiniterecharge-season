/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretRotate extends CommandBase {
  TurretSubsystem m_turret;

  DoubleSupplier m_input;

  /**
   * Rotate the turret linearly by use a supplier.
   * 
   * @param input A supplier/lambda that gives a double from a joystick or other input.
   */
  public TurretRotate(TurretSubsystem turret, DoubleSupplier input) {
    m_turret = turret;
    m_input = input;
    addRequirements(this.m_turret);
  }
  
  @Override
  public void initialize() {
  }
  
  @Override
  public void execute() {
    m_turret.rotate(m_input.getAsDouble());
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
  
  @Override
  public void end(boolean interrupted) {
    m_turret.stop();
  }
}