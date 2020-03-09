/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DashboardContainer;

/**
 * A subsystem that contains ONLY the compressor for the pneumatics.
 * @see ClimbSubsystem
 * @see DiskSubsystem
 */
public class PneumaticsSubsystem extends SubsystemBase {

  private Compressor compressor;

  /**
   * Creates a new Pneumatics.
   */
  public PneumaticsSubsystem() {
    compressor = new Compressor();
    compressor.start();
  }
  
  /**
	 * Allows the compressor to pump air at low pressures (not all the time).
	 */
	public void enableCompressor() {
		compressor.start();
	}

	/**
	 * Prevents the compressor from pumping air, at any time.
	 */
	public void disableCompressor() {
		compressor.stop();
	}

  @Override
  public void periodic() {
    putDashboard();
  }

  public void putDashboard() {
    switch (DashboardContainer.getInstance().getVerbosity()) {
      case 5:
        SmartDashboard.putBoolean("Compressor/Pressurized", !compressor.getPressureSwitchValue());
        SmartDashboard.putBoolean("Compressor/Enabled", compressor.enabled());
      case 4:
      case 3:
      case 2:
      case 1:
        break;
      default:
        break;
    }
  }
}
