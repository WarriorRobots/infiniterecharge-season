package frc.robot.commands.autonomous.paths;

import frc.robot.commands.autonomous.paths.PathBuilder.Waypoint;
import frc.lib.control.Path;

import java.util.ArrayList;

public class CargoShipFrontToFeederPath implements PathContainer {
    public static final String kLookForTargetMarker = "LOOK_FOR_TARGET";

    boolean mLeft;

    public CargoShipFrontToFeederPath(boolean left) {
        mLeft = left;
    }

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(142.5, (mLeft ? 1.0 : -1.0) * -45, 0, 0));
        sWaypoints.add(new Waypoint(100, (mLeft ? 1.0 : -1.0) * 20, 40, 100));
        sWaypoints.add(new Waypoint(55, (mLeft ? 1.0 : -1.0) * 80, 0, 100, kLookForTargetMarker));
        sWaypoints.add(new Waypoint(-5, (mLeft ? 1.0 : -1.0) * 80, 0, 100));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return true;
    }
}
