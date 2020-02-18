/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.spinner;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSpinnerSubsystem;

public class SpinToColor extends CommandBase {
  /**
   * Creates a new SpinToColor.
   */
  
   /** The color the field wants, set when the command is initialized. */
  String desiredColor;

  /**
   * A boolean to keep track whether the wheel is finding the color or is turning it so
   * the field sees the desired color.
   * True 
   */
  int phase;

  /** True is clockwise, false is counterclockwise */
  boolean direction = true;
  
  /** JOSE CODE JOSE CODE */

  ColorSpinnerSubsystem spinner;
  
  /**
   * Spins the color wheel to a specific color provided by the field.
   * The desired color will be set when the command is run since the field must provide the color.
   */
  public SpinToColor(ColorSpinnerSubsystem spinner)
   {
     this.spinner = spinner;
    // Use addRequirements() here to declare subsystem dependencies.
    // Not so sure about this one
    addRequirements(this.spinner);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    // get the color from the field.
    //desiredColor = <get from field>
    desiredColor = DriverStation.getInstance().getGameSpecificMessage();
    switch (desiredColor) {
      case "B": 
        phase = 1;
        desiredColor = "G";
        break;
      case "G":
        phase = 1;
        desiredColor = "B";
        break; 
      case "R":
        phase = 2;
        desiredColor = "G";
        break;
      case "Y":
        phase = 2;
        desiredColor = "B";
        break;
    
      default:
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    // CURRENTLY IN PHASE ONE WORKING TO PHASE TWO
    if (phase == 1 && spinner.getColor() != desiredColor)
    {
      spinner.spinWheel(0.25);
    } else if (phase == 1) {
      phase = 2;
      if (desiredColor.equals("B"))
      {
        direction = false;
      }
      desiredColor = "Y";
    }
     // CURRENTLY IN PHASE ONE WORKING TO FINISH
     if (phase == 2 && spinner.getColor() != desiredColor)
     {
       spinner.spinWheel(0.25 * (direction ? 1: -1));
     }
     // if the desired color is not found, rotate the wheel till it is found 
     // if the desired color is found, rotate the wheel so the field sees the desired color 
     // if the robot sees the desired color, the desired color is found 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    // True when the robot has made it so the field sensor can see the desired color
    // JOSE CODE JOSE CODE
    return (desiredColor == "") || (phase == 2 && spinner.getColor() == desiredColor);
  }
}
