package org.usfirst.frc.team3310.paths.auton;

import java.util.ArrayList;

import org.usfirst.frc.team3310.paths.PathBuilder;
import org.usfirst.frc.team3310.paths.PathBuilder.Waypoint;
import org.usfirst.frc.team3310.paths.PathContainer;
import org.usfirst.frc.team3310.utility.control.Path;
import org.usfirst.frc.team3310.utility.math.RigidTransform2d;
import org.usfirst.frc.team3310.utility.math.Rotation2d;
import org.usfirst.frc.team3310.utility.math.Translation2d;


public class RightStartToScaleRightV2 implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(18,51,0,130));
        sWaypoints.add(new Waypoint(40,51,0,130,        "shiftHi"));
        sWaypoints.add(new Waypoint(145,52,55,130,                   "raiseElevator"));
        sWaypoints.add(new Waypoint(257,93,0,130,              "startEject"));
        sWaypoints.add(new Waypoint(282,100,0,130));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(18, 51), Rotation2d.fromDegrees(180)); 
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
}