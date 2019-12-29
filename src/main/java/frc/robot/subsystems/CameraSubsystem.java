package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.Constants;
import frc.robot.QuickAccessVars;
import frc.robot.commands.ChangeCameraPipeline;

/**
 * CameraSubsystem is supposed to recive data from the limelight to be output or processed.
 */
public class CameraSubsystem extends Subsystem {

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

	// TODO Fix pipelines on the limelight and in the code
	/** Pipeline id for the crosshair in the center. */
	public static final int PIPELINE_CENTER = 0;
	/** Pipeline id for Driver exposure and use */
	public static final int PIPELINE_DRIVER = 1;

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
	 * Gets the percentage area of the currently-seen object relative to the image size. 
	 * @return Decimal representing percentage of image taken up by object, 0 to 1.
	 */
	public double getObjectArea() {
		return visionTable.getEntry(TARGET_AREA).getDouble(0);
	}

	/**
	 * Gets the rotation angle of the currently-seen object.
	 * @return -90 degrees to 90 degrees
	 */
	public double getObjectRotationAngle() {
		return visionTable.getEntry(TARGET_SKEW).getDouble(0);
	}

	/**
	 * @return Aspect ratio of width / height; -1 if can not see the target.
	 */
	public double getObjectWidthRange() {

		double width = visionTable.getEntry(TARGET_WIDTH).getDouble(0);
		double range = getTargetDistance();

		return (canSeeObject()) ? width / range : -1;
	}

	/**
	 * Finds the distance to the target depending on the ratio of the width to height.
	 * (If ratio>CUTTING_RATIO, uses width to find distance as part of the target is cutting.)
	 * (If ratio<=CUTTING_RATIO, uses height to find distance as the target is perfect or off center.)
	 * See page 7 of the programming book for more details.
	 * @return Distance from target in inches; -1 if can not see the target.
	 */
	public double getTargetDistance() {

		if (!canSeeObject())
			return -1;

		double height, width, angle, range;
		
		// height in pixels
		height = visionTable.getEntry(TARGET_HEIGHT).getDouble(0);

		// width in pixels
		width = visionTable.getEntry(TARGET_WIDTH).getDouble(0);

		// if ratio > CUTTING_RATIO, use width to calculate distance
		if (width/height>Constants.CUTTING_RATIO) {
			// angle in radians
			angle = width / Constants.PPR_H;

			// range = adj = opp/tan(Theta/2)
			range = (Constants.TARGET_WIDTH/2) / Math.tan(angle/2);
		}

		// if ratio <= CUTTING_RATIO, use height to calculate distance
		else
		{
			// angle in radians
			angle = height / Constants.PPR_V;

			// range = adj = opp/tan(Theta)
			range = Constants.TARGET_HEIGHT / Math.tan(angle);
		}

		return range;
	}

	/**
	 * (Adjacent because the range is the adjacent side.)
	 * Uses the conversion from pixels to radians and uses radians to figures out the trigonometry
	 * between the height of the target and the range.
	 * (Target may not be visible at very close range, use by Width)
	 * @return Distance from target in inches; -1 if can not see the target.
	 * @see #getTargetDistance
	 */
	public double getTargetDistanceByHeight() {

		if (!canSeeObject())
			return -1;

		// height in pixels
		double height = visionTable.getEntry(TARGET_HEIGHT).getDouble(0);

		// angle in radians
		double angle = height / Constants.PPR_V;

		// range = adj = opp/tan(Theta)
		double range = Constants.TARGET_HEIGHT / Math.tan(angle);

		return range;
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

	@Override
	public void initDefaultCommand() {
		setDefaultCommand(new ChangeCameraPipeline(PIPELINE_DRIVER));
	}

}
