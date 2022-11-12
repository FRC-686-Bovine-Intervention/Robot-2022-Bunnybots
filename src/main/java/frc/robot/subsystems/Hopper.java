package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

public class Hopper extends Subsystem {
    private static Hopper instance;

    public static Hopper getInstance() {
        if (instance == null) {
            instance = new Hopper();
        }
        return instance;
    }

    // Hardware
    private final CANSparkMax blanketMotor;
    private final DoubleSolenoid leftFlapSolenoid, rightFlapSolenoid;
    private static final boolean invertLeftSolenoid = false;
    private static final boolean invertRightSolenoid = false;

    private static final double kBlanketMotorPower = 0.1;
    private static final double kCalibrationPower = 0.1;
    private static final double kCalPosition = 8;

    private Hopper() {
        // Initalize
        blanketMotor = new CANSparkMax(Constants.kBlanketMotorID, MotorType.kBrushless);
        leftFlapSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
                Constants.kLeftFlapSolenoidFChannel,
                Constants.kLeftFlapSolenoidRChannel);
        rightFlapSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
                Constants.kRightFlapSolenoidFChannel,
                Constants.kRightFlapSolenoidRChannel);

        // Configure
        blanketMotor.restoreFactoryDefaults();
        blanketMotor.setInverted(false);
        blanketMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)kCalPosition);
        blanketMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

        calibrated = false;
    }

    private boolean blanketUp = false;

    private boolean leftFlapOpen = false;
    private boolean rightFlapOpen = false;

    @Override
    public void run() {
        blanketMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        blanketMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        if(calibrated && autoCalibrate)
        {
            blanketMotor.set((blanketUp ? 1 : -1) * kBlanketMotorPower);
        }
        else
        {
            blanketMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
            blanketMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
            blanketMotor.set(kCalibrationPower);
        }

        if(blanketMotor.getOutputCurrent() >= 5)
        {
            blanketMotor.getEncoder().setPosition(kCalPosition);
            calibrated = true;
        }

        leftFlapSolenoid.set(leftFlapOpen ^ invertLeftSolenoid ? Value.kForward : Value.kReverse);
        rightFlapSolenoid.set(rightFlapOpen ^ invertRightSolenoid ? Value.kForward : Value.kReverse);
    }

    public Hopper setBlanket(boolean up) {
        blanketUp = up;
        return this;
    }

    public Hopper setFlaps(boolean left, boolean right) {
        leftFlapOpen = left;
        rightFlapOpen = right;
        return this;
    }

    private ShuffleboardTab tab = Shuffleboard.getTab("Hopper");
    private NetworkTableEntry blanketEntry = tab.add("Blanket Up", false).withWidget(BuiltInWidgets.kBooleanBox)             .withPosition(1,1).getEntry();
    private NetworkTableEntry leftFlapEntry = tab.add("Left Flap Open", false).withWidget(BuiltInWidgets.kBooleanBox)         .withPosition(0,0).withSize(2,1).getEntry();
    private NetworkTableEntry rightFlapEntry = tab.add("Right Flap Open", false).withWidget(BuiltInWidgets.kBooleanBox)   .withPosition(0,1).getEntry();
    private NetworkTableEntry calibratedEntry = tab.add("Calibrated", false).withWidget(BuiltInWidgets.kBooleanBox)             .withPosition(1,1).getEntry();

    @Override
    public void updateShuffleboard() {
        blanketEntry.setBoolean(blanketUp);
        leftFlapEntry.setBoolean(leftFlapOpen);
        rightFlapEntry.setBoolean(rightFlapOpen);
        calibratedEntry.getBoolean(calibrated);
    }
}
