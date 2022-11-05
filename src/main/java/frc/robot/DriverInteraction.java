package frc.robot; 
 
import frc.robot.controls.Controls;
import frc.robot.controls.Controls.ButtonControlEnum;
import frc.robot.lib.util.RisingEdgeDetector;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.ArmPosEnum;
import frc.robot.subsystems.Intake.IntakeState; 
 
public class DriverInteraction { 
    private static DriverInteraction instance; 
    public static DriverInteraction getInstance() {if(instance == null){instance = new DriverInteraction();}return instance;} 
 
    // final Drive drive; 
    final Controls controls; 
    final Intake intake; 
 
    private DriverInteraction() 
    { 
        controls = Controls.getInstance(); 
        // drive = Drive.getInstance(); 
        intake = Intake.getInstance(); 
    } 
 
    public void init() {
    }

    final RisingEdgeDetector ClawButtonEdgeDetector = new RisingEdgeDetector();
    final RisingEdgeDetector IntakeButtonEdgeDetector = new RisingEdgeDetector();
 
    public void run() 
    {
        ClawButtonEdgeDetector.update(controls.getButton(ButtonControlEnum.CLAW_GRAB));
        IntakeButtonEdgeDetector.update(controls.getButton(ButtonControlEnum.INTAKE_NEXT_STATE));
        switch(intake.intakeStatus)
        {
            case GROUND:
                if(controls.getButton(ButtonControlEnum.CLAW_GRAB))
                    intake.setState(IntakeState.GRAB);
            break;
            case GRAB:
                if(!controls.getButton(ButtonControlEnum.CLAW_GRAB) && intake.isAtPos(ArmPosEnum.GROUND))
                    intake.setState(IntakeState.GROUND);
                else if(IntakeButtonEdgeDetector.get())
                    intake.setState(IntakeState.RAISED);
            break;
            case RAISED:
                if(IntakeButtonEdgeDetector.get())
                    intake.setState(IntakeState.DROP);
                else if (ClawButtonEdgeDetector.get())
                    intake.setState(IntakeState.GRAB);
            break;

            default:    break;
        }
        // drive.setOpenLoop(controls.getDriveCommand()); 
    } 
} 
