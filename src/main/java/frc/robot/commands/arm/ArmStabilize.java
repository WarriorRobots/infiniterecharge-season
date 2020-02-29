/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

public class ArmStabilize extends InstantCommand {

  private ArmSubsystem m_arm;

  /**
  public ArmStabilize(ArmSubsystem arm) {
   * @param arm Arm subsystem
   */
  public ArmStabilize(ArmSubsystem arm) {
    m_arm = arm;
    addRequirements(m_arm);
  }

  @Override
  public void initialize() {
    m_arm.rotateToPositionNoSafety(m_arm.getPosition());
  }
}
