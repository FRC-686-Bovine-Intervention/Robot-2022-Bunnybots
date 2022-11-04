package frc.robot.auto.modes;

import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.DriveDistanceAction;

public class WheelPositionAuto extends AutoMode{
    public WheelPositionAuto(){}

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new DriveDistanceAction(+70, -70));
        runAction(new DriveDistanceAction(-70, +70));
    }
}
