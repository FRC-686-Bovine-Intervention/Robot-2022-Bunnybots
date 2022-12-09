package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
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
    private final Solenoid leftFlapSolenoid, rightFlapSolenoid;
    private static final boolean invertLeftSolenoid = false;
    private static final boolean invertRightSolenoid = false;

    private static final double kBlanketMotorPower = 1;
    private static final double kCalibrationPower = 0.25;
    private static final double kCalPosition = 650;

    private static final double kDisableCoastTimeThreshold = 5;

    private Hopper() {
        // Initalize
        blanketMotor = new CANSparkMax(Constants.kBlanketMotorID, MotorType.kBrushless);
        leftFlapSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,
                Constants.kLeftFlapSolenoidFChannel);
        rightFlapSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,
                Constants.kRightFlapSolenoidFChannel);

        // Configure
        blanketMotor.restoreFactoryDefaults();
        blanketMotor.setInverted(false);
        blanketMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)kCalPosition-25);
        blanketMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

        calibrated = false;
    }

    private boolean blanketUp = false;

    private boolean leftFlapOpen = false;
    private boolean rightFlapOpen = false;

    @Override
    public void run() {
        disabledInit = true;
        
        if(calibrated)
        {
            blanketMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
            blanketMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
            blanketMotor.setOpenLoopRampRate(0.25);
            blanketMotor.set((blanketUp ? 1 : -1) * kBlanketMotorPower);
        }
        else if(autoCalibrate)
        {
            blanketMotor.setOpenLoopRampRate(1);
            blanketMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
            blanketMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
            blanketMotor.set(kCalibrationPower);
            if(blanketMotor.getOutputCurrent() >= 5)
            {
                blanketMotor.getEncoder().setPosition(kCalPosition);
                calibrated = true;
            }
        }

        leftFlapSolenoid.set(leftFlapOpen ^ invertLeftSolenoid);
        rightFlapSolenoid.set(rightFlapOpen ^ invertRightSolenoid);
    }

    private boolean disabledInit = true;
    private double disabledTime;
    @Override
    public void disable()
    {
        if(disabledInit)
        {
            disabledTime = Timer.getFPGATimestamp();
        }
        if(Timer.getFPGATimestamp() - disabledTime > kDisableCoastTimeThreshold)
        {
            blanketMotor.setIdleMode(IdleMode.kCoast);
            calibrated = false;
        }
        disabledInit = false;
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
    private NetworkTableEntry blanketEntry = tab.add("Blanket Up", false).withWidget(BuiltInWidgets.kBooleanBox)        .withPosition(0,1).withSize(2,1).getEntry();
    private NetworkTableEntry leftFlapEntry = tab.add("Left Flap Open", false).withWidget(BuiltInWidgets.kBooleanBox)   .withPosition(0,0).getEntry();
    private NetworkTableEntry rightFlapEntry = tab.add("Right Flap Open", false).withWidget(BuiltInWidgets.kBooleanBox) .withPosition(1,0).getEntry();
    private NetworkTableEntry calibratedEntry = tab.add("Calibrated", false).withWidget(BuiltInWidgets.kBooleanBox)     .withPosition(0,2).getEntry();
    private NetworkTableEntry calibrateButton = tab.add("Calibrate", false).withWidget(BuiltInWidgets.kToggleButton)    .withPosition(1,2).getEntry();
    private NetworkTableEntry motorOutputEntry = tab.add("Motor Current", -10000).withWidget(BuiltInWidgets.kTextView)  .withPosition(7,0).getEntry();
    private NetworkTableEntry motorPosEntry = tab.add("Motor Pos", -10000).withWidget(BuiltInWidgets.kTextView)         .withPosition(8,0).getEntry();

    @Override
    public void updateShuffleboard() {
        blanketEntry.setBoolean(blanketUp);
        leftFlapEntry.setBoolean(leftFlapOpen);
        rightFlapEntry.setBoolean(rightFlapOpen);
        calibratedEntry.setBoolean(calibrated);
        motorOutputEntry.setDouble(blanketMotor.getOutputCurrent());
        motorPosEntry.setDouble(blanketMotor.getEncoder().getPosition());
        if (calibrateButton.getBoolean(false))
        {
            calibrated = false;
            calibrateButton.setBoolean(false);
        }
    }
}
