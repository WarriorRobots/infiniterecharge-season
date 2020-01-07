/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class TurretRotate extends Command {

  private DoubleSupplier input;

  /**
   * Rotate the turret linearly by use a supplier.
   * 
   * @param input A supplier/lambda that gives a double from a joystick or other input.
   */
  public TurretRotate(DoubleSupplier input) {
    requires(Robot.turret);
    this.input = input;
  }
  
  @Override
  protected void initialize() {
  }
  
  @Override
  protected void execute() {
    Robot.turret.rotate(input.getAsDouble());
  }
  
  @Override
  protected boolean isFinished() {
    return false;
  }
  
  @Override
  protected void end() {
    Robot.turret.stop();
  }
  
  @Override
  protected void interrupted() {
    Robot.turret.stop();
  }
}
