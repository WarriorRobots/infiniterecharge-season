/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.control.Lookahead;
import frc.lib.control.Path;
import frc.lib.control.PathFollower;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Twist2d;
import frc.lib.util.DriveSignal;
import frc.robot.Constants;
import frc.robot.IO;
import frc.robot.Vars;
import frc.robot.util.Kinematics;
import frc.robot.util.RobotState;

public class DrivetrainSubsystem extends SubsystemBase {
  
  private static final int ID_FRONTLEFT = 14;
  private static final int ID_BACKLEFT = 15;
  private static final int ID_FRONTRIGHT = 1;
  private static final int ID_BACKRIGHT = 0;

  private WPI_TalonFX FrontLeft, BackLeft, FrontRight, BackRight;
  
  private SpeedControllerGroup LeftGroup, RightGroup;
  
  private DifferentialDrive drive;
  
  private AHRS navx;

  private Path currentPath = null;
	private PathFollower pathFollower;
	private DriveControlStates currentDriveControlState;
  
  /**
   * Resolution of the encoders.
   * @see https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-resolution
   */
  private static final int CLICKS_PER_REV = 2048;
  
  /**
   * Equivilant to 1/Gear Ratio.
   * Use this to convert from 1 rotation of the motor to 1 rotation of the output shaft: input * GEARING = output.
   */
  private static final double GEARING = 12.0/50.0 * 20.0/54.0;
  
  private static final double WHEEL_DIAMETER = 6;
  
  /** The distance between the left and right wheels is {@value} inches. */
  public static final double TRACK_WIDTH = 26.0;
  
  /** The robot wheel is {@value} inches in radius. */
  public static final double WHEEL_RADIUS = WHEEL_DIAMETER / 2.0;
  
  /** The distance between the left and right wheels is {@value} meters. */
  public static final double TRACK_WIDTH_METER = TRACK_WIDTH / 2.0 * 0.0254;
  
  /** The robot's track's scrub factor. (unitless) */
    public static final double SCRUB_FACTOR = 1.0469745223; // TODO Change this from 254 to 2478 scrub factor
  
  public DrivetrainSubsystem() {
    
    FrontLeft = new WPI_TalonFX(ID_FRONTLEFT);
    BackLeft = new WPI_TalonFX(ID_BACKLEFT);
    FrontRight = new WPI_TalonFX(ID_FRONTRIGHT);
    BackRight = new WPI_TalonFX(ID_BACKRIGHT);

    FrontLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.PRIMARY_PID, Constants.MS_TIMEOUT);
    FrontRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.PRIMARY_PID, Constants.MS_TIMEOUT);
    FrontLeft.setSensorPhase(false);
    FrontRight.setSensorPhase(true);

    LeftGroup = new SpeedControllerGroup(FrontLeft, BackLeft);
    RightGroup = new SpeedControllerGroup(FrontRight, BackRight);
    LeftGroup.setInverted(false);
    RightGroup.setInverted(false);

    drive = new DifferentialDrive(LeftGroup, RightGroup);

    // if NavX is missing, this code will handle errors and prevent a crash
		try {
      navx = new AHRS(I2C.Port.kMXP);
		} catch (Exception e) {
			DriverStation.reportError(e.getMessage(), true);
    }
    
    // Drive state starts out as Open loop, following driver commands or voltage commands
		currentDriveControlState = DriveControlStates.OPEN_LOOP;
  }

  static class PERIODICio {

    static double angle = 0; // degrees

    static int leftEnc = 0; // native units
    static int rightEnc = 0; // native units

    static int leftEncVelocity = 0; // native units / 100ms
    static int rightEncVelocity = 0; // native units / 100ms

  }

  /**
   * Drive with tankdrive with raw values to the sides for control systems that are non-human.
   * @param left Left speed from -1 to 1.
   * @param right Right speed from -1 to 1.
   */
  public void tankdriveRaw(double left, double right) {
    drive.tankDrive(left, right);
  }

  /**
   * Drive with tankdrive with squared inputs for human drivers.
   * @param left Left speed from -1 to 1.
   * @param right Right speed from -1 to 1.
   */
  public void tankdriveSquared(double left, double right) {
    drive.tankDrive(left, right);
  }

  /**
   * @return the native units of the left encoder.
   */
  public static int getLeftEnc() {
    return PERIODICio.leftEnc;
  }

  /**
   * @return the native units of the right encoder.
   */
  public static int getRightEnc() {
    return PERIODICio.rightEnc;
  }

  /**
   * @return The inches of the left encoder.
   */
  public static double getLeftPosition() {
    // clicks * rev/clicks * output/input = revs
    // revs * PI * diameter = distance
    return (double) PERIODICio.leftEnc / CLICKS_PER_REV * GEARING * Math.PI * WHEEL_DIAMETER;
  }

  /**
   * @return The inches position of the right encoder.
   */
  public static double getRightPosition() {
    // clicks * rev/clicks * output/input = revs
    // revs * PI * diameter = distance
    return (double) PERIODICio.rightEnc / CLICKS_PER_REV * GEARING * Math.PI * WHEEL_DIAMETER;
  }

  /**
   * @return The inches/second of the left encoder.
   */
  public static double getLeftVelocity() {
    // clicks/100ms * 10(100ms/s) * rev/clicks * output/input = rev/s
    // revs/s * PI * diameter = veloicity (in/s)
    return (double) PERIODICio.leftEncVelocity * 10 / CLICKS_PER_REV * GEARING * Math.PI * WHEEL_DIAMETER;
  }

  /**
   * @return The inches/second of the right encoder.
   */
  public static double getRightVelocity() {
    // clicks/100ms * 10(100ms/s) * rev/clicks * output/input = rev/s
    // revs/s * PI * diameter = veloicity (in/s)
    return (double) PERIODICio.rightEncVelocity * 10 / CLICKS_PER_REV * GEARING * Math.PI * WHEEL_DIAMETER;
  }

  /**
	 * Gets yaw angle of the robot.
   * @return angle in degrees.
	 */
	public static double getAngleDegrees() {
		return PERIODICio.angle;
	}

	/**
	 * Gets yaw angle of the robot.
   * @return angle in radians.
	 */
	public double getAngleRadians() {
		return PERIODICio.angle * (Math.PI / 180.0);
  }

  /**
   * Zeros the drivetrain yaw angle.
   */
  public void resetAngle() {
    navx.zeroYaw();
  }
  
  /**
   * Zeros the left drivetrain encoder.
   */
  public void resetLeftEnc() {
    FrontLeft.setSelectedSensorPosition(0);
  }
  
  /**
   * Zeros the right drivetrain encoder.
   */
  public void resetRightEnc() {
    FrontRight.setSelectedSensorPosition(0);
  }

  /**
   * Zeros the drivetrain encoders.
   */
  public void resetEnc() {
    resetLeftEnc();
    resetRightEnc();
  }
  
  /**
   * Zeros ALL the sensors affiliated with the drivetrain.
   */
  public void reset() {
    resetAngle();
    resetEnc();
  }

  /*----------------------------------------------------------------------------------*/
	/* MIT License                                                                      */
	/*                                                                                  */
	/* Copyright (c) 2019 Team 254                                                      */
	/*                                                                                  */
	/* Permission is hereby granted, free of charge, to any person obtaining a copy     */
	/* of this software and associated documentation files (the "Software"), to deal    */
	/* in the Software without restriction, including without limitation the rights     */
	/* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell        */
	/* copies of the Software, and to permit persons to whom the Software is            */
	/* furnished to do so, subject to the following conditions:                         */
	/*                                                                                  */
	/* The above copyright notice and this permission notice shall be included in all   */
	/* copies or substantial portions of the Software.                                  */
	/*                                                                                  */
	/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR       */
	/* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,         */
	/* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE      */
	/* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER           */
	/* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,    */
	/* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE    */
	/* SOFTWARE.                                                                        */
	/*----------------------------------------------------------------------------------*/

	/**
	 * Sets drivetrain to follow this path.
     *
	 * @see Path
     */
	/* (See liscence above) */
  public synchronized void setWantDrivePath(Path path, boolean reversed) {
    if (currentPath != path || currentDriveControlState != DriveControlStates.PATH_FOLLOWING) {
      RobotState.getInstance().resetDistanceDriven();
        pathFollower = new PathFollower(path, reversed, new PathFollower.Parameters(
          new Lookahead(Vars.kMinLookAhead, Vars.kMaxLookAhead, Vars.kMinLookAheadSpeed,
              Vars.kMaxLookAheadSpeed),
            Vars.kInertiaSteeringGain, Vars.kPathFollowingProfileKp,
            Vars.kPathFollowingProfileKi, Vars.kPathFollowingProfileKv,
            Vars.kPathFollowingProfileKffv, Vars.kPathFollowingProfileKffa,
            Vars.kPathFollowingProfileKs, Vars.kPathFollowingMaxVel,
            Vars.kPathFollowingMaxAccel, Vars.kPathFollowingGoalPosTolerance,
            Vars.kPathFollowingGoalVelTolerance, Vars.kPathStopSteeringDistance));
        currentDriveControlState = DriveControlStates.PATH_FOLLOWING;
        currentPath = path;
      } else {
        stopDrive();
      }
  }

  /**
  * @return True if the path is finished or true if the robot is not following a path.
  */
  /* (See liscence above) */
  public synchronized boolean isDoneWithPath() {
    if (currentDriveControlState == DriveControlStates.PATH_FOLLOWING && pathFollower != null) {
      return pathFollower.isFinished();
    } else {
      System.out.println("Robot is not in path following mode");
      return true;
    }
  }

  /**
  * Starts the process to end the path if following a path, otherwise does nothing.
  */
  /* (See liscence above) */
  public synchronized void forceDoneWithPath() {
    if (currentDriveControlState == DriveControlStates.PATH_FOLLOWING && pathFollower != null) {
      pathFollower.forceFinish();
    } else {
      System.out.println("Robot is not in path following mode");
    }
  }

  /**
  * Called every loop to update the path following for robot.
  * 
  * @param timestamp A timestamp that describes how long the robot has been on or describes the current time. (Used to find the difference in time.)
  */
  /* (See liscence above) */
  private void updatePathFollower(double timestamp) {
    if (currentDriveControlState == DriveControlStates.PATH_FOLLOWING) {
        RobotState robot_state = RobotState.getInstance();
        Pose2d field_to_vehicle = robot_state.getLatestFieldToVehicle().getValue();
        Twist2d command = pathFollower.update(timestamp, field_to_vehicle, robot_state.getDistanceDriven(),
          robot_state.getPredictedVelocity().dx);
        if (!pathFollower.isFinished()) {
          DriveSignal setpoint = Kinematics.inverseKinematics(command);
          tankdriveRaw(setpoint.getLeft(), setpoint.getRight());
        } else {
          if (!pathFollower.isForceFinished()) {
            stopDrive();
          }
        }
    } else {
      DriverStation.reportError("drive is not in path following state", false);
    }
  }

  public void setOpen() {
    currentDriveControlState = DriveControlStates.OPEN_LOOP;
  }

  /**
  * Shuts off all drive motors.
  */
  public void stopDrive() {
    drive.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (IO.verbose) putDashboard();

    synchronized (DrivetrainSubsystem.this) {
			if (currentDriveControlState == DriveControlStates.PATH_FOLLOWING && currentPath != null) {
				updatePathFollower(Timer.getFPGATimestamp());
			}
		}

    PERIODICio.angle = navx.getAngle();
    PERIODICio.leftEnc = FrontLeft.getSelectedSensorPosition();
    PERIODICio.rightEnc = FrontRight.getSelectedSensorPosition();
    PERIODICio.leftEncVelocity = FrontLeft.getSelectedSensorVelocity();
    PERIODICio.rightEncVelocity = FrontRight.getSelectedSensorVelocity();

  }

  /**
   * Puts information about this subsystem on the dashboard.
   */
  public void putDashboard() {
    SmartDashboard.putNumber("Navx Degrees", getAngleDegrees());
    SmartDashboard.putNumber("Navx Radians", getAngleRadians());
    SmartDashboard.putNumber("Left encoder", getLeftEnc());
    SmartDashboard.putNumber("Right encoder", getRightEnc());
    SmartDashboard.putNumber("Left position (in)", getLeftPosition());
    SmartDashboard.putNumber("Right position (in)", getRightPosition());
    SmartDashboard.putNumber("Left veloicity (in/s)", getLeftVelocity());
    SmartDashboard.putNumber("Right veloicity (in/s)", getRightVelocity());
  }

	private enum DriveControlStates {
		OPEN_LOOP, // following controls from driver or velocities
		PATH_FOLLOWING, // following controls from path
	}
}
