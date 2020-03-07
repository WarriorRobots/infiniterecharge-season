/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Vars;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.TurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TurretAimSequence extends ParallelCommandGroup {
  /**
   * Creates a new TurretAimSequence.
   */
  public TurretAimSequence(CameraSubsystem camera, TurretSubsystem turret, ArmSubsystem arm, boolean finishable) {
    super(
      new ArmToPosition(arm, Vars.ARM_SHOOTING),
      new TurretAim(camera, turret){public boolean isFinished(){return finishable;}}
    );
  }
}
