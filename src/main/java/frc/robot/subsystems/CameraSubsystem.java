package frc.robot.subsystems;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.Constants;
import frc.robot.commands.camera.CameraChangePipeline;

/**
 * CameraSubsystem is supposed to recive data from the limelight to be output or processed.
 */
public class CameraSubsystem extends Subsystem {
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

	// DELETED getObjectRotationalAngle

	// DELETED getObjectWIdthRange()
	
	// JOSE CODE JOSE CODE WITH LANCE MATH LANCE MATH

	// HUGE NOTE: CAN CHOOSE BETWEEN "port" AND "pickup" 
	// MOST LIKELY AIMING FOR PORT SO THAT IS WHAT WILL BE USED
	public double TargetDistance(String target)
	{
		if (canSeeObject())
		{
			// defines the true dimensions based on the true height and width of the selected targe
			double trueWidth = 0;
			double aspectRatio = 0;
			if(target == "port")
			{
				trueWidth = Constants.PORT_WIDTH;
				aspectRatio = Constants.PORT_ASPECT_RATIO;
			}
			else if(target == "pickup")
			{
				trueWidth = Constants.PICKUP_WIDTH;
				aspectRatio = Constants.PICKUP_ASPECT_RATIO;
			}
			
			// defines the pixel height and pixel width of the object
			double pixelHeight = getObjectHeight();
			double pixelWidth = getObjectWidth();
			// corrects the width of the target so that it can be used to calculate the distance
			if(aspectRatio * pixelHeight > pixelWidth)
			{
				pixelWidth = aspectRatio * pixelHeight;
			}
			// imports the pixel distance to the target
			double pixelDistance = Constants.PIXEL_DISTANCE;
			// uses the ratio inches/pixels to convert the pixel distance into inches
			return pixelDistance * (trueWidth / pixelWidth);
		}
		return -1;
	}

	public double TargetOffsetAngle()
	{
		return visionTable.getEntry(TARGET_X).getDouble(0);
		
	}

	// DELETED getTargetDistanceByHeight

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
		setDefaultCommand(new CameraChangePipeline(PIPELINE_DRIVER));
	}

}
