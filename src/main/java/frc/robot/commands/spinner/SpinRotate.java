/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.spinner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSpinnerSubsystem;

public class SpinRotate extends CommandBase {
  /**
   * Creates a new SpinRotate.
   */
  
  /**
   * This command spins the color wheel 4 rotations.
   * (This means it will pass by the same start color 8 times.)
   */
  int count;
  String startingColor;
  boolean spunAlready;

  ColorSpinnerSubsystem whee;

  public SpinRotate(ColorSpinnerSubsystem whee) {
    this.whee = whee;
    // Use addRequirements() here to declare subsystem dependencies.
    // Not so sure about this one
    addRequirements(this.whee);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startingColor = whee.getColor();
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
     // Rotate the wheel four times
     if (count != 8)
     {
       whee.spinWheel(0.25);
       if (whee.getColor() == startingColor && !spunAlready) // spunAlready != true is the same thing 
       {
         spunAlready = true;
         count++;
       }
       if (whee.getColor() != startingColor)
       {
         spunAlready = false;
       }
     }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // JOSE CODE JOSE CODE
  // Get tagged
  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    // returns whether the wheel has been spun 4 times
    System.out.println("Spun 4 times lol");
    return count >= 8;
  }
}