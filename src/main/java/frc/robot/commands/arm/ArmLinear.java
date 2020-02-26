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
  
  ArmSubsystem m_arm;
  DoubleSupplier m_input;

  /**
   * Creates a new ArmRotate.
   * To ~90 degrees
   * @param armInput -1 to 1 for voltage to arm
   */
  public ArmLinear(ArmSubsystem arm, DoubleSupplier input) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    addRequirements(this.m_arm);
    m_input = input;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.rotateAtPercent(m_input.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
