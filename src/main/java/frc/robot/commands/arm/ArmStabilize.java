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

public class ArmStabilize extends CommandBase {

  private ArmSubsystem m_arm;

  private double m_position;

  /**
   * Tell the robot to make the arm stay where it is.
   * @param arm Arm subsystem
   */
  public ArmStabilize(ArmSubsystem arm) {
    m_arm = arm;
    addRequirements(m_arm);
  }

  @Override
  public void initialize() {
    m_position = m_arm.getPosition();
  }
  
  @Override
  public void execute() {
    m_arm.rotateToPositionNoSafety(m_position);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_arm.getPosition() - m_position) < Vars.ARM_TOLERANCE;
  }
}
