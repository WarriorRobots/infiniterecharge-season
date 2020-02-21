/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.Vars;

public class ArmSubsystem extends SubsystemBase {

  public static final double CLICKS_PER_DEGREE = 34.0;
  
  private WPI_TalonSRX m_arm;
  
  /**
   * Creates a new ArmSubsystem.
   */
  public ArmSubsystem() {
    m_arm = new WPI_TalonSRX(RobotMap.ID_ARM);

		m_arm.setInverted(Vars.ARM_REVERSED);
		m_arm.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.PRIMARY_PID, Constants.MS_TIMEOUT);
		m_arm.setSensorPhase(Vars.ARM_ENCODER_REVERSED);

		m_arm.config_kP(Constants.PRIMARY_PID, Vars.ARM_P, Constants.MS_TIMEOUT);
    m_arm.config_kI(Constants.PRIMARY_PID, 0, Constants.MS_TIMEOUT);
    
    // limitSwitch = new DigitalInput(LIMIT_SWITCH_PORT);

  }

  public void rotateAtPercent(double percent)
  {
    m_arm.set(ControlMode.PercentOutput, percent);
  }
  
  public void rotateToPosition(double degrees)
  {
    m_arm.set(ControlMode.Position, toClicks(degrees));
		if (belowMinimum(degrees)) {
      m_arm.set(ControlMode.Position, toClicks(Vars.ARM_MINIMUM_ANGLE));
			System.out.println("Arm moving to " + degrees + ", cutting short to prevent crash!");
		} else if (aboveMaximum(degrees)) {
			m_arm.set(ControlMode.Position, toClicks(Vars.ARM_MAXIMUM_ANGLE));
			System.out.println("Arm moving to " + degrees + ", cutting short to prevent crash!");
		} else {
			m_arm.set(ControlMode.Position, toClicks(degrees));
		}
  }

  /**
	 * Converts degrees to encoder clicks.
	 * @param degrees Angle measured from the output axle.
	 */
	public int toClicks(double degrees) {
		return (int) Math.round(degrees * CLICKS_PER_DEGREE);
	}

  /**
	 * Returns true if the specified angle is above the upper bound of motion.
	 * <p>True is BAD. Use this in code to avoid crashing the arm.
	 * @param degrees Any degree measurement related to the arm.
	 */
	private boolean aboveMaximum(double degrees) {
		return degrees > Vars.ARM_MAXIMUM_ANGLE;
  }
  
  /**
	 * Returns true if the specified angle is below the lower bound of motion.
	 * <p>True is BAD. Use this in code to avoid crashing the arm.
	 * @param degrees Any degree measurement related to the arm.
	 */
	private boolean belowMinimum(double degrees) {
		return degrees < Vars.ARM_MINIMUM_ANGLE;
	}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
