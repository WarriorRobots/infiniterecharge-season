/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;

public class HopperPower extends CommandBase {
  HopperSubsystem m_hopper;
  double m_hopperwall_percent, m_hopperfloor_percent;
  /**
   * Run hopper at some desired percent.
   * @param hopper Hopper subsystem
   * @param hopperwall_percent Percent output of wall when running this command.
   * @param hopperfloor_percent Percent output of floor when running this command.
   */
  public HopperPower(HopperSubsystem hopper, double hopperwall_percent, double hopperfloor_percent) {
    m_hopper = hopper;
    addRequirements(this.m_hopper);
    m_hopperwall_percent = hopperwall_percent;
    m_hopperfloor_percent = hopperfloor_percent;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_hopper.setWallPower(m_hopperwall_percent);
    m_hopper.setFloorPower(m_hopperfloor_percent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hopper.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
