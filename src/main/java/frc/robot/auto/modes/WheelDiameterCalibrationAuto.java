package frc.robot.auto.modes;

import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.PathFollowerAction;
import frc.robot.command_status.RobotState;
import frc.robot.lib.util.Path;
import frc.robot.lib.util.Path.Waypoint;
import frc.robot.lib.util.PathSegment.Options;
import frc.robot.lib.util.Vector2d;

public class WheelDiameterCalibrationAuto extends AutoMode{
    public WheelDiameterCalibrationAuto(){}

    @Override
    protected void routine() throws AutoModeEndedException {
        // drive slow so we don't slip the wheels
        double maxSpeed = 24;
        double accel = 12;
        double lookaheadDist = 48;
        boolean visionEnabled = false;
        Options driveOptions = new Options(maxSpeed, accel, lookaheadDist, visionEnabled);

        double distanceToDriveFeet = 16.0;
        Vector2d initialPos = initialPose.getPosition();
        Vector2d finalPos = initialPos.add(new Vector2d(distanceToDriveFeet*12,0));
        Path path = new Path();
        path.add(new Waypoint(initialPos, driveOptions));
        path.add(new Waypoint(  finalPos, driveOptions));
        
        runAction(new PathFollowerAction(path));

        // print out the distance the robot thinks it has traveled.  Adjust DriveLoop.kDriveWheelCircumInches until they match
        System.out.println("Measure the distance the robot has travelled with a tape measure.");
        System.out.println("Compare with the distance the robot thinks it has travelled, below.");
        System.out.println("Adjust DriveLoop.kDriveWheelCircumInches until they match");
        System.out.printf("RobotState distance travelled = %.1f inches\n", RobotState.getInstance().getLatestFieldToVehicle().getPosition().distance(initialPos));
    }
}
