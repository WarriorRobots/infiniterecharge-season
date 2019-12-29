/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LedControllerSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;

/**
 * Main class of the Robot.
 */
public class Robot extends TimedRobot {

  public static final CameraSubsystem camera = new CameraSubsystem();
  public static final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
  public static final LedControllerSubsystem leds = new LedControllerSubsystem();
  public static final PneumaticSubsystem pneumatics = new PneumaticSubsystem();

  /** Reference this to get input from the joysticks and Xbox controller. */
	public static ControlHandler input;
  
  @Override
  public void robotInit() {
    input = new ControlHandler();
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}
  
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testPeriodic() {}
}
