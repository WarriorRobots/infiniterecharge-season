/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.IO;
import frc.robot.Vars;
/**
 * CameraSubsystem is supposed to recive data from the limelight to be output or processed.
 */
public class CameraSubsystem extends SubsystemBase {
	// This is not an itersubsystem so that the camera subsystem is not told to do the angle calculation every tick
	// This may later change so that during targeting, the camera does the calculation once per tick vs once per call (where it may be called several times per tick)

	/** Limelight network table keyword */
	private static final String LIMELIGHT = "limelight";
	/** Whether the limelight has any valid targets (0 or 1) */
	private static final String TARGET_EXISTS = "tv";
	/** Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees) */
	private static final String TARGET_X = "tx";
	/** Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees) */
	private static final String TARGET_Y = "ty";
	/** Target Area (0% of image to 100% of image) */
	private static final String TARGET_AREA = "ta";
	/** Skew or rotation (-90 degrees to 0 degrees) */
	private static final String TARGET_SKEW = "ts";
	/** Horizontal sidelength of the rough bounding box (0 - 320 pixels) */
	private static final String TARGET_WIDTH = "thor";
	/** Vertical sidelength of the rough bounding box (0 - 320 pixels) */
	private static final String TARGET_HEIGHT = "tvert";




	/** Vision table for Limelight */
	private NetworkTable visionTable;

	/** Pipeline id for Driver exposure and use */
	public static final int PIPELINE_DRIVER = 0;
	/** Pipeline id for aiming at the target in general. */
	public static final int PIPELINE_HEX = 1;
	/** Pipeline id for aiming at the target using 3d. */
	public static final int PIPELINE_3D = 2;

	public CameraSubsystem() {
		visionTable = NetworkTableInstance.getDefault().getTable(LIMELIGHT);
	}

	/**
	 * Checks for an object in the Limelight's frame.
	 * @return {@code true} if object is present, else {@code false}
	 */
	public boolean canSeeObject() {
		return (visionTable.getEntry(TARGET_EXISTS).getDouble(0) == 1) ? true : false;
	}

	// CAN'T GET RID OF X AND Y

	/**
	 * Gets x-coordinate of current object on screen.
	 * @return X position of object in pixels
	 */
	public double getObjectX() {
		return visionTable.getEntry(TARGET_X).getDouble(0);
	}

	/**
	 * Gets y-coordinate of current object on screen.
	 * @return Y position of object in pixels
	 */
	public double getObjectY() {
		return visionTable.getEntry(TARGET_Y).getDouble(0);
	}

	/**
	 * Gets the width of the current object on the screen.
	 * @return Pixel width of object.
	 */
	public double getObjectWidth() {
		return visionTable.getEntry(TARGET_WIDTH).getDouble(0);
	}

	/**
	 * Gets the height of the current object on the screen.
	 * @return Pixel height of object.
	 */
	public double getObjectHeight() {
		return visionTable.getEntry(TARGET_HEIGHT).getDouble(0);
	}

	/**
	 * Gets the percentage area of the currently-seen object relative to the image size. 
	 * @return Decimal representing percentage of image taken up by object, 0 to 1.
	 */
	public double getObjectArea() {
		return visionTable.getEntry(TARGET_AREA).getDouble(0);
	}

	/**
	 * Gets the skew of the object.
	 * @return Degrees rotated from -90 to 0.
	 */
	public double getObjectSkew() {
		return visionTable.getEntry(TARGET_SKEW).getDouble(0);
	}
	
	/**
	 * Get the distance from the target in inches. (The hexagonal target.)
	 * (Returns -1 if target is not visible.)
	 * @author Joshua
	 * @return The distance from the target in inches.
	 */
	public double getTargetDistance() {
		if (!canSeeObject()) return -1;

		double numerator = Constants.TARGET_ELEVATION - Vars.ELEVATION;
		double radians =
			Math.toRadians(
				Vars.CAMERA_TILT +
				visionTable.getEntry(TARGET_Y).getDouble(0)
			) -
			visionTable.getEntry(TARGET_HEIGHT).getDouble(0) / Constants.PPR_V / 2;
		double denominator = Math.tan(radians);

		double distance = numerator / denominator;
		return distance;
	}

	/**
	 * Gives the height of the target seen by the camera
	 * @return Height of object in pixels
	 */
	public double getTargetHeight() {
		return visionTable.getEntry(TARGET_HEIGHT).getDouble(0);
	}

	/** Changes the pipeline the Limelight is using to a new specified pipeline.
	 * @param pipeline The pipeline the Limelight to change to (0 to 9).
	 */
	public void setPipeline(int pipeline) {

		// if pipeline is NOT between 0 and 9, warn the dashboard and quit the command.
		if (!(0 <= pipeline && pipeline <= 9)) {
			DriverStation.reportWarning(Integer.toString(pipeline) + " is not a valid pipeline to change to.", true);
			// leave function to prevent a real error from the limelight
			return;
		}
		visionTable.getEntry("pipeline").setDouble(pipeline);

	}

	/** Grabs the pipeline the Limelight is using currently */
	public int getPipeline() {

		return (int) visionTable.getEntry("getpipe").getDouble(-1.0);

	}

	@Override
	public void initSendable(SendableBuilder builder) {
		// builder.setSmartDashboardType("subsystem-camera");
		// builder.addBooleanProperty("object-exists", () -> canSeeObject(), null);
		// builder.addDoubleArrayProperty("object-x&y", () -> {

		// 	return new double[] { getObjectX(), getObjectY() };

		// }, null);
		// builder.addDoubleProperty("object-x", () -> getObjectX(), null);
		// builder.addDoubleProperty("object-y", () -> getObjectX(), null);
		// builder.addDoubleProperty("object-area", () -> getObjectArea(), null);
		// builder.addDoubleProperty("object-skew", () -> getObjectRotationAngle(), null);

		// builder.addDoubleProperty("object-ratio", () -> getObjectAspectRatio(), null);
		// builder.addDoubleProperty("object-distance", () -> getTargetDistance(), null);
		// builder.addDoubleProperty("object-widthrange", () -> getObjectWidthRange(), null);
	}

	public void putDashboard() {
		switch (IO.verbose) {
			case 5:
				SmartDashboard.putBoolean("Camera/Object Visible", canSeeObject());
			case 4:
			case 3:
				SmartDashboard.putNumber("Camera/Object X", getObjectX());
			case 2:
			case 1:
				SmartDashboard.putNumber("Camera/Target Distance", getTargetDistance());
				SmartDashboard.putBoolean("Camera/In 9-15\"", (9 * 12 <= getTargetDistance() && getTargetDistance() <= 15 * 12));


			break;
			default:
				break;
		}
	}
}
