package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.ArmPosEnum;
import frc.robot.subsystems.Intake.IntakeState;

public class EngageIntakeAction implements Action{
    private Intake intake = Intake.getInstance();

    private final double startTime;
    private boolean cratesDropped = false;
    private boolean ballsDumped = false;

    public EngageIntakeAction() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void start() {
        intake.setState(IntakeState.GRAB);
    }
    
    @Override
    public void run() {
        if(intake.intakeStatus == IntakeState.DROP)
            cratesDropped = true;
        if(!ballsDumped && Timer.getFPGATimestamp() - startTime > 0.5)
        {
            intake.setState(IntakeState.DUMP);
            ballsDumped = true;
        }
    }

    @Override
    public boolean isFinished() {
        return cratesDropped && intake.isAtPos(ArmPosEnum.GROUND);
    }

    @Override
    public void done() {
        
    }
}
