/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.camera;

// JOSHUA HELP

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vars;
import frc.robot.commands.led.LEDChangePattern;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LedControllerSubsystem;

public class CameraStopAtDistance extends CommandBase {
  /** PID used for approaching the target. */
  // TODO include replacement for PIDs

  DrivetrainSubsystem m_drive;
  CameraSubsystem m_snapsnap;
  LedControllerSubsystem m_scan;
  private SynchronousPIDF PIDapproach;
  /** PID for keeping the target centered */
  private SynchronousPIDF PIDcenter;

  private Timer timer;
  /** Calculated PID output from {@link #PIDapproach} should stored in value. */
  private double valueApproach;
  /** Calculated PID output from {@link #PIDcenter} should stored in value. */
  private double valueCenter;

  // states whether the command can be finished or not
  private boolean finishable;

  
  
  /**
    * @param finishable True if the command can be stopped by being at a distance (aka true if in auto).
    */
  public CameraStopAtDistance(DrivetrainSubsystem drive, CameraSubsystem snapsnap, LedControllerSubsystem scan, boolean finishable) {
    m_drive = drive;
    addRequirements(this.m_drive);
    m_snapsnap = snapsnap;
    addRequirements(this.m_snapsnap);
    m_scan = scan;
    addRequirements(this.m_scan);

    valueApproach = 0;
    valueCenter = 0;
    
    PIDapproach = new SynchronousPIDF(Vars.KP_APPROACH, Vars.KI_APPROACH, Vars.KD_APPROACH);
    PIDcenter = new SynchronousPIDF(Vars.KP_CENTER, Vars.KI_CENTER, Vars.KD_CENTER);
    
    this.finishable = finishable;
    timer = new Timer();
  }
  
  @Override
  public void initialize() {
    System.out.println("Camera: Starting " + this.getClass().getSimpleName());
    m_snapsnap.setPipeline(CameraSubsystem.PIPELINE_CENTER);

    // distance in in inches
    PIDapproach.setSetpoint(Vars.SETPOINT_APPROACH);
    PIDcenter.setSetpoint(0); // 0 means keep the target centered

    timer.start();
  }

  @Override
  public void execute() {
    
    if (m_snapsnap.canSeeObject()) {
    valueApproach = PIDapproach.calculate(m_snapsnap.getTargetDistance(), timer.get());
    valueCenter = PIDcenter.calculate(m_snapsnap.getObjectX() + Vars.CAMERA_BIAS, timer.get());

    // if the robot within the specified range then RED STROBING
    if (Math.abs(m_snapsnap.getTargetDistance() - Vars.SETPOINT_APPROACH) < Vars.TOLERANCE_APPROACH) {
      m_scan.setChannel(LedControllerSubsystem.atTarget);
    }
    // if the robot sees the target however is not where it should be BLUE STROBING
    else {
      m_scan.setChannel(LedControllerSubsystem.seeTarget);
     }
    } else {
      valueCenter = 0;
      valueApproach = 0;
      
      // if the robot doesn't see the target it should be BLUE BREATHING
      m_scan.setChannel(LedControllerSubsystem.IDLE);
    }
    /** TODO replace the arcadeDriveRaw */
    
    m_drive.arcadeDriveRaw(-valueApproach, -valueCenter);
  }
  
  @Override
  public boolean isFinished() {
    if (finishable) {
      if (Math.abs(m_snapsnap.getTargetDistance() - Vars.SETPOINT_APPROACH) < Vars.TOLERANCE_APPROACH) {
        // returns true if the command is finishable and the camera is at the right distance away
        return true;
      }
    }
    // returns false if the commmand is not finishable or if it is not at the tolerable zone to say it's finished
    return false;
  }
  
  @Override
  public void end(boolean interrupted) {
    System.out.println("Camera: Finishing " + this.getClass().getSimpleName());
    timer.stop();
    PIDapproach.reset();
    PIDcenter.reset();
    valueApproach = 0;
    valueCenter = 0;
    // TODO include stopDrive
    m_drive.stopDrive();
  }
}