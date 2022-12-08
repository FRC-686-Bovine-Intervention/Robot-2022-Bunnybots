package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;

public class SetIntakeStateAction implements Action{
    private Intake intake = Intake.getInstance();
    private final IntakeState state;
    private final double startTime;

    public SetIntakeStateAction(IntakeState state) {
        this.state = state;
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void start() {
    }
    
    @Override
    public void run() {
        intake.setState(state);
    }

    @Override
    public boolean isFinished() {
        return intake.isAtPos(state.armPos) && Timer.getFPGATimestamp() - startTime >= 0.5;
    }

    @Override
    public void done() {
        
    }
}
