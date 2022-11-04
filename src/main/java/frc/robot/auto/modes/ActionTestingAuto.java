package frc.robot.auto.modes;

import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.PathFollowerAction;
import frc.robot.auto.actions.WaitAction;
import frc.robot.lib.util.Path;
import frc.robot.lib.util.Path.Waypoint;
import frc.robot.lib.util.PathSegment.Options;
import frc.robot.lib.util.Vector2d;

public class ActionTestingAuto extends AutoMode{
    public ActionTestingAuto(){}

    @Override
    protected void routine() throws AutoModeEndedException {
        Path rPath = new Path();
        Path fPath = new Path();

        double length = 60;
        double width = 24;

        Vector2d initPos = new Vector2d(0,0);
        Vector2d halfPos = new Vector2d(-length/2,0);
        Vector2d halfPos2 = new Vector2d(-length/2,-width);
        Vector2d finalPos = new Vector2d(-length,0);
        Options driveOptions = new Options(48,72,36,false);

        rPath.add(new Waypoint(initPos, driveOptions));
        rPath.add(new Waypoint(halfPos, driveOptions));
        rPath.add(new Waypoint(halfPos2, driveOptions));
        rPath.setReverseDirection();

        fPath.add(new Waypoint(halfPos2, driveOptions));
        fPath.add(new Waypoint(halfPos, driveOptions));
        fPath.add(new Waypoint(finalPos, driveOptions));

        runAction(new PathFollowerAction(rPath));
        runAction(new WaitAction(0.1));
        runAction(new PathFollowerAction(fPath));
    }
}
