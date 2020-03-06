/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vars;
import frc.robot.subsystems.ArmSubsystem;

public class ArmZero extends CommandBase {
  
  ArmSubsystem m_arm;

  /**
   * Make the arm go from some forwards position and move back to the hall effect to set the zero.
   */
  public ArmZero(ArmSubsystem arm) {
    m_arm = arm;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.rotateAtPercent(Vars.ARM_RESET_PERCENT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.rotateToPosition(m_arm.getPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.hallEffect();
  }
}
