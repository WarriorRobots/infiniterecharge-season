/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.Vars;

public class FeedSubsystem extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  private WPI_TalonSRX m_feed;
  public FeedSubsystem() {
    m_feed = new WPI_TalonSRX(RobotMap.ID_FEED);
    m_feed.setInverted(Vars.FEED_REVERSED);
  }

  // Spins the motor at some low value and other for a specific percent, 

  public void feedAtPercent(double voltage)
  {
    m_feed.set(ControlMode.PercentOutput, voltage);
  }

  public void stop() {
    m_feed.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
