package frc.robot.commands.autonomous.paths;

import frc.robot.commands.autonomous.paths.PathBuilder.Waypoint;
import frc.lib.control.Path;

import java.util.ArrayList;

public class Hab1ToCargoShip1Path implements PathContainer {
    public static final String kStartAutoAimingMarker = "START_AUTO_AIMING";

    boolean mLeft;

    public Hab1ToCargoShip1Path(boolean left) {
        mLeft = left;
    }

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(0, 0.0, 0, 0));
        sWaypoints.add(new Waypoint(30, 0.0, 0, 40.0, kStartAutoAimingMarker));
        sWaypoints.add(new Waypoint(110, (mLeft ? 1.0 : -1.0) * 1.5, 1, 100.0, kStartAutoAimingMarker));
        sWaypoints.add(new Waypoint(205, (mLeft ? 1.0 : -1.0) * 3, 0, 120));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}