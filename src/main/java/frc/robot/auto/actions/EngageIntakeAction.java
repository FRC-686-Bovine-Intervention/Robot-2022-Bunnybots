package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.ArmPosEnum;
import frc.robot.subsystems.Intake.IntakeState;

public class EngageIntakeAction implements Action{
    private Intake intake = Intake.getInstance();

    private double grabTime;
    private enum IntakeStep {
        CALIBRATING,
        GRABBING,
        GRABBED,
        DUMPING,
        DROPPED,
        FINISHED
    }
    private IntakeStep intakeStep = IntakeStep.CALIBRATING;

    public EngageIntakeAction() {
        
    }

    @Override
    public void start() {
    }
    
    @Override
    public void run() {
        switch(intakeStep)
        {
            case CALIBRATING:
                if(intake.calibrated)
                    intakeStep = IntakeStep.GRABBING;
            break;
            case GRABBING:
                if(intake.isAtPos(ArmPosEnum.GROUND))
                {
                    grabTime = Timer.getFPGATimestamp();
                    intakeStep = IntakeStep.GRABBED;
                    break;
                }
                intake.setState(IntakeState.GROUND);
            break;
            case GRABBED:
                intake.setState(IntakeState.GRAB);
                if(Timer.getFPGATimestamp() - grabTime > 0.5)
                {
                    intakeStep = IntakeStep.DUMPING;
                    intake.setState(IntakeState.DUMP);
                }
            break;
            case DUMPING:
                if(intake.isAtPos(ArmPosEnum.DROP))
                    intakeStep = IntakeStep.DROPPED;
            break;
            case DROPPED:
                if(intake.isAtPos(ArmPosEnum.GROUND))
                    intakeStep = IntakeStep.FINISHED;
            break;
            default: break;
        }
    }

    @Override
    public boolean isFinished() {
        return intakeStep == IntakeStep.FINISHED;
    }

    @Override
    public void done() {
        
    }
}
