package frc.robot.commands.autonomous.paths;

import frc.robot.commands.autonomous.paths.PathBuilder.Waypoint;
import frc.lib.control.Path;

import java.util.ArrayList;

public class CargoShip1ToFeederPath implements PathContainer {
    public static final String kLookForTargetMarker = "LOOK_FOR_TARGET";

    boolean mLeft;

    public CargoShip1ToFeederPath(boolean left) {
        mLeft = left;
    }

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(205, (mLeft ? 1.0 : -1.0) * 5, 0, 120));
        sWaypoints.add(new Waypoint(105, (mLeft ? 1.0 : -1.0) * 40, 35, 100));
        sWaypoints.add(new Waypoint(55, (mLeft ? 1.0 : -1.0) * 75, 0, 120, kLookForTargetMarker));
        sWaypoints.add(new Waypoint(-20, (mLeft ? 1.0 : -1.0) * 75, 0, 120));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return true;
    }
}
