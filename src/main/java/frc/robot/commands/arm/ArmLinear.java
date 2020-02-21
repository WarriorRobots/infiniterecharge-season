/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmLinear extends CommandBase {
  /**
   * Creates a new ArmRotate.
   * To ~90 degrees
   * @param armInput -1 to 1 for voltage to arm
   */
  ArmSubsystem m_monkey;
  DoubleSupplier m_armInput;

/**
   * Creates a new setHopperPower.
   * @param wallInput -1 to 1 for voltage to floor/wall
   */
  public ArmLinear(ArmSubsystem monkey, DoubleSupplier armInput) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_monkey = monkey;
    addRequirements(this.m_monkey);
    m_armInput = armInput;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_monkey.rotateToPosition(m_armInput.getAsDouble());

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
