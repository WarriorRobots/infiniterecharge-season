/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
  
  private static final int ID_FRONTLEFT = 14;
  private static final int ID_BACKLEFT = 15;
  private static final int ID_FRONTRIGHT = 1;
  private static final int ID_BACKRIGHT = 0;

  private WPI_TalonFX FrontLeft, BackLeft, FrontRight, BackRight;

  private SpeedControllerGroup LeftGroup, RightGroup;

  private DifferentialDrive drive;

  public DrivetrainSubsystem() {

    FrontLeft = new WPI_TalonFX(ID_FRONTLEFT);
    BackLeft = new WPI_TalonFX(ID_BACKLEFT);
    FrontRight = new WPI_TalonFX(ID_FRONTRIGHT);
    BackRight = new WPI_TalonFX(ID_BACKRIGHT);

    LeftGroup = new SpeedControllerGroup(FrontLeft, BackLeft);
    RightGroup = new SpeedControllerGroup(FrontRight, BackRight);

    drive = new DifferentialDrive(LeftGroup, RightGroup);

  }

  public void tankdrive(double left, double right) {
    drive.tankDrive(left, right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
