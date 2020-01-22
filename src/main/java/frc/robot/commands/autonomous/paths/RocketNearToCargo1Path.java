package frc.robot.commands.autonomous.paths;

import frc.robot.commands.autonomous.paths.PathBuilder.Waypoint;
import frc.lib.control.Path;

import java.util.ArrayList;

public class RocketNearToCargo1Path implements PathContainer {

    boolean mLeft;

    public RocketNearToCargo1Path(boolean left) {
        mLeft = left;
    }

    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(125, (mLeft ? 1.0 : -1.0) * 75, 0, 120));
        sWaypoints.add(new Waypoint(145, (mLeft ? 1.0 : -1.0) * 40, 30, 120));
        sWaypoints.add(new Waypoint(200, (mLeft ? 1.0 : -1.0) * -5, 0, 120));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }

    @Override
    public boolean isReversed() {
        return false;
    }
}
