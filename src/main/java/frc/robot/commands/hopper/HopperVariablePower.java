/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hopper;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;

public class HopperVariablePower extends CommandBase {
  HopperSubsystem m_hippityhop;
  DoubleSupplier m_wallInput, m_floorInput;
  /**
   * Creates a new HopperPower.
   * @param wallInput -1 to 1 for voltage to wall
   * @param floorInput -1 to 1 for voltage to floor
   */
  public HopperVariablePower(HopperSubsystem hippityhop, DoubleSupplier wallInput, DoubleSupplier floorInput) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_hippityhop = hippityhop;
    addRequirements(this.m_hippityhop);
    m_wallInput = wallInput;
    m_floorInput = floorInput;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_hippityhop.setWallPower(m_wallInput.getAsDouble());
    m_hippityhop.setFloorPower(m_floorInput.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
