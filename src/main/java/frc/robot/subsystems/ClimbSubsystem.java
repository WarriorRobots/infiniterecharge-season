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

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DashboardContainer;
import frc.robot.RobotMap;
import frc.robot.Vars;

/**
 * A subsystem containing a motor to pull the pulley to make the climb extend and
 * a piston to activate the brakes for the climb.
 */
public class ClimbSubsystem extends SubsystemBase {

  /**
   * Resolution of the encoders.
   * @see https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-resolution
   */
  private static final int CLICKS_PER_REV = 4096;

  /**
   * Equivilant to 1/Gear Ratio.
   * Use this to convert from 1 rotation of the motor to 1 rotation of the output shaft: input * GEARING = output.
   */
  private static final double GEARING = 20.0/40.0;

  private WPI_TalonSRX m_winch;

  private DoubleSolenoid m_brakes;

  public ClimbSubsystem() {
    m_winch = new WPI_TalonSRX(RobotMap.ID_CLIMB);

    m_winch.setInverted(Vars.CLIMB_REVERSED);
    m_winch.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.PRIMARY_PID, Constants.MS_TIMEOUT);
    m_winch.setSensorPhase(Vars.CLIMB_ENCODER_REVERSED);

    m_winch.config_kP(Constants.PRIMARY_PID, Vars.CLIMB_P, Constants.MS_TIMEOUT);

    m_brakes = new DoubleSolenoid(RobotMap.ID_BRAKES_F, RobotMap.ID_BRAKES_B);
    m_brakes.set(DoubleSolenoid.Value.kOff);
  }

  /**
   * Give a percent to the winch motor.
   * (NOTE: THIS IS UNSAFE AS THE CLIMB COULD RUN PAST WHERE IT IS INTENDED TO.)
   */
  public void setPercent(double percent) {
    m_winch.set(ControlMode.PercentOutput, percent);
  }

  /**
   * Give a position for the climb to go to.
   * @param position Desired position of the physical climb in inches upwards.
   */
  public void setPosition(double position) {
    if (position < Vars.CLIMB_MINIMUM) {
      m_winch.set(ControlMode.Position, toClicks(Vars.CLIMB_MINIMUM));
    } else if (position > Vars.CLIMB_MAXIMUM) {
      m_winch.set(ControlMode.Position, toClicks(Vars.CLIMB_MAXIMUM));
    } else {
      m_winch.set(ControlMode.Position, toClicks(position));
    }
  }

  /**
   * Give a position for the climb to go to.
   * (NOTE: THIS WILL NOT STOP THE CLIMB FROM BEING COMMANDED PAST IT'S PHYSICAL STOPS)
   * @param position Desired position of the physical climb in inches upwards.
   */
  public void setPositionUnsafe(double position) {
    m_winch.set(ControlMode.Position, toClicks(position));
  }

  /**
   * Get the encoder of the winch.
   * @return Number of clicks on the encoder
   */
  public int getEnc() {
    return m_winch.getSelectedSensorPosition();
  }

  /**
   * Get the position of the physical climb.
   * @return Displacement of the climb upwards in inches.
   */
  public double getPosition() {
    return toInches(m_winch.getSelectedSensorPosition());
  }

  /**
   * Convert inches of movement of the climb into clicks of movement of the motor.
   * @param inches Number of inches of upwards motion of the climb.
   * @return Number of clicks of rotation of the motor.
   */
  public int toClicks(double inches) {
    return (int) Math.floor(inches / Math.PI / Vars.CLIMB_TRACK_DIAMETER / GEARING * CLICKS_PER_REV);
  }

  /**
   * Convert clicks of movement of the motor into inches of movement of the climb.
   * @param clicks Number of clicks of rotation of the motor.
   * @return Number of Clicks of rotation of the motor.
   */
  public double toInches(double clicks) {
    return clicks / CLICKS_PER_REV * GEARING * Math.PI * Vars.CLIMB_TRACK_DIAMETER;
  }

  /**
   * Set the value of the brakes. <p>
   * @param value A {@link DoubleSolenoid.Value}
   * @see #stopBrakes
   * @see #engageBrakes
   * @see #disengageBrakes
   */
  public void setBrakes(Brakes state) {
    m_brakes.set(state.getValue());
  }

  /**
   * kOff -> None/stop <p>
   * kReverse -> Engage brakes <p>
   * kForward -> Disengage brakes <p>
   */
  public static enum Brakes {
    stop(DoubleSolenoid.Value.kOff),
    engage(DoubleSolenoid.Value.kReverse),
    disengage(DoubleSolenoid.Value.kForward);
    public DoubleSolenoid.Value value;
    Brakes(DoubleSolenoid.Value value) {
      this.value = value;
    }
    public DoubleSolenoid.Value getValue() {
      return value;
    }
  }

  /**
   * Used to stop giving pressure to the brakes in either direction.
   * This neither remove nor adds pressure, therefore maintaining the previous state.
   * This must be called after several loops of the forwards as to not overwork the pneumatics.
   */
  public void stopBrakes() {
    setBrakes(Brakes.stop);
  }

  /**
   * Pulls brakes into braking device.
   */
  public void engageBrakes() {
    setBrakes(Brakes.engage);
  }

  /**
   * Pushes brake out of the braking device.
   */
  public void disengageBrakes() {
    setBrakes(Brakes.disengage);
  }

  /**
   * Used to stop the climb.
   * This needs to be used to make sure the climb does not move when the brakes are going to be applied.
   */
  public void stopWinch() {
    m_winch.stopMotor();
  }

  /**
   * The stop command is private because the two components of the climb should be stopped
   * individually as there is no time either should be stopped in unison.
   */
  @SuppressWarnings("unused")
  private void stop() {}

  @Override
  public void periodic() {
    putDashboard();
  }

  /**
   * Puts information about this subsystem on the dashboard.
   */
  public void putDashboard() {
    switch (DashboardContainer.getInstance().getVerbosity()) {
      case 5:
        SmartDashboard.putNumber("Climb/Get gain", m_winch.getMotorOutputPercent());
        SmartDashboard.putNumber("Climb/Encoder", getEnc());
			case 4:
      case 3:
        SmartDashboard.putString("Climb/Brakes", m_brakes.get().toString());
      case 2:
        SmartDashboard.putNumber("Climb/Position", getPosition());
      case 1:
			  break;
			default:
        break;
    }
  }
}
