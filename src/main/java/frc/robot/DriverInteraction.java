package frc.robot;

import frc.robot.controls.Controls;
import frc.robot.controls.Controls.ButtonControlEnum;
import frc.robot.lib.util.RisingEdgeDetector;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.ArmPosEnum;
import frc.robot.subsystems.Intake.IntakeState;

public class DriverInteraction {
    private static DriverInteraction instance;

    public static DriverInteraction getInstance() {
        if (instance == null) {
            instance = new DriverInteraction();
        }
        return instance;
    }

    final Drive drive;
    final Controls controls;
    final Intake intake;
    final Hopper hopper;

    private DriverInteraction() {
        controls = Controls.getInstance();
        drive = Drive.getInstance();
        intake = Intake.getInstance();
        hopper = Hopper.getInstance();
    }

    public void init() {
    }

    private final RisingEdgeDetector ClawButtonEdgeDetector = new RisingEdgeDetector();
    private final RisingEdgeDetector IntakeButtonEdgeDetector = new RisingEdgeDetector();
    private final RisingEdgeDetector DefenseEdgeDetector = new RisingEdgeDetector();

    public void run() {
        ClawButtonEdgeDetector.update(controls.getButton(ButtonControlEnum.CLAW_GRAB));
        IntakeButtonEdgeDetector.update(controls.getButton(ButtonControlEnum.INTAKE_NEXT_STATE));
        DefenseEdgeDetector.update(controls.getButton(ButtonControlEnum.DEFENSE));
        switch (intake.intakeStatus) {
            case DEFENSE:
                if (DefenseEdgeDetector.get())
                    intake.setState(IntakeState.GROUND);
            break;
            case GROUND:
                if (ClawButtonEdgeDetector.get())
                    intake.setState(IntakeState.GRAB);
                if (DefenseEdgeDetector.get())
                    intake.setState(IntakeState.DEFENSE);
                break;
            case GRAB:
                if (!controls.getButton(ButtonControlEnum.CLAW_GRAB) && intake.isAtPos(ArmPosEnum.GROUND))
                    intake.setState(IntakeState.GROUND);
                else if (IntakeButtonEdgeDetector.get())
                    intake.setState(IntakeState.DUMP);
                break;
            case DUMP:
                if (IntakeButtonEdgeDetector.get())
                    intake.setState(IntakeState.DROP);
                else if (ClawButtonEdgeDetector.get())
                    intake.setState(IntakeState.GRAB);
                break;

            default:
                break;
        }
        drive.setOpenLoop(controls.getDriveCommand());

        hopper.setBlanket(controls.getButton(ButtonControlEnum.BLANKET))
                .setFlaps(controls.getButton(ButtonControlEnum.LEFT_FLAP),
                        controls.getButton(ButtonControlEnum.RIGHT_FLAP));
    }
}
