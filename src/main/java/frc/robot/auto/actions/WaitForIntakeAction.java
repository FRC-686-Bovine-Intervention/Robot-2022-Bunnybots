package frc.robot.auto.actions;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.ArmPosEnum;

public class WaitForIntakeAction implements Action{
    private Intake intake = Intake.getInstance();
    private final ArmPosEnum pos;

    public WaitForIntakeAction(ArmPosEnum pos) {
        this.pos = pos;
    }

    @Override
    public void start() {
    }
    
    @Override
    public void run() {
    }

    @Override
    public boolean isFinished() {
        return intake.isAtPos(pos) && intake.calibrated;
    }

    @Override
    public void done() {
        
    }
}
