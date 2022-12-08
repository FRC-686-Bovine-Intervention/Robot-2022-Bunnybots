package frc.robot.auto.modes;

import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.EngageIntakeAction;
import frc.robot.auto.actions.PathFollowerAction;
import frc.robot.auto.actions.SetIntakeStateAction;
import frc.robot.auto.actions.WaitForIntakeAction;
import frc.robot.command_status.RobotState;
import frc.robot.lib.util.Path;
import frc.robot.lib.util.Path.Waypoint;
import frc.robot.lib.util.PathSegment.Options;
import frc.robot.loops.DriveLoop;
import frc.robot.subsystems.Intake.ArmPosEnum;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.lib.util.Pose;
import frc.robot.lib.util.Vector2d;

public class EpicAwesomeAutoV4868_0 extends AutoMode{
    public EpicAwesomeAutoV4868_0(){}

    @Override
    protected void routine() throws AutoModeEndedException {
        final int kRowsOfCrates = 4;
        final double kReverseInches = 2 + DriveLoop.kPathFollowingCompletionTolerance;

        Path initPath = new Path();
        Path rPath = new Path();

        Vector2d initPos = new Vector2d(0,0);
        Options driveOptions = new Options(48,96,36,false);

        initPath.add(new Waypoint(initPos, driveOptions));
        initPath.add(new Waypoint(FieldDimensions.firstCratePos, driveOptions));
        rPath.add(new Waypoint(FieldDimensions.firstCratePos, driveOptions));
        rPath.add(new Waypoint(FieldDimensions.firstCratePos.sub(new Vector2d(kReverseInches, 0)), driveOptions));
        rPath.setReverseDirection();

        RobotState.getInstance().reset(new Pose());

        runAction(new WaitForIntakeAction(ArmPosEnum.GROUND));
        runAction(new PathFollowerAction(initPath));
        runAction(new SetIntakeStateAction(IntakeState.GRAB));
        runAction(new PathFollowerAction(rPath));
        runAction(new EngageIntakeAction());
        Waypoint lastWaypoint = new Waypoint(FieldDimensions.firstCratePos.sub(new Vector2d(kReverseInches, 0)), driveOptions);
        for(int i = 1; i < kRowsOfCrates; i++)
        {
            Path nextCratePath = new Path();
            Path reversePath = new Path();
            nextCratePath.add(lastWaypoint);    // Add previous waypoint as initial waypoint
            lastWaypoint = new Waypoint(lastWaypoint.position.add(new Vector2d(FieldDimensions.crateWidth + kReverseInches + 2,0)), driveOptions);   // Generate next waypoint
            nextCratePath.add(lastWaypoint);    // Add next waypoint
            reversePath.add(lastWaypoint);
            lastWaypoint = new Waypoint(lastWaypoint.position.sub(new Vector2d(kReverseInches,0)), driveOptions);   // Generate next waypoint
            reversePath.add(lastWaypoint);
            reversePath.setReverseDirection();
            runAction(new PathFollowerAction(nextCratePath));
            runAction(new SetIntakeStateAction(IntakeState.GRAB));
            runAction(new PathFollowerAction(reversePath));
            runAction(new EngageIntakeAction());
        }
        System.out.println("Auto Done");
    }
}
