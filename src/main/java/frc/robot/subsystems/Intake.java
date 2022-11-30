package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

public class Intake extends Subsystem{
    private static Intake instance;
    public static Intake getInstance() {if(instance == null){instance = new Intake();} return instance;}

    // Hardware
    private final TalonFX leftArmMotor, rightArmMotor;
    private final DigitalInput calSwitch;
    private final Solenoid clawSolenoid;
    private static final boolean invertSolenoid = true;

    // Calibration
    private static final double kCalibrationPercentOutput = 0.25;
    
    // Disable Behavior
    private static final double kDisableCoastTimeThreshold = 5;
    private static final double kDisableRecalPosThreshold = 1;

    // Unit Conversion
    private static final double kGearRatio = 300;  // 16 in gearbox, 48t:12t sprockets
    private static final double kEncoderUnitsPerRev = 2048 * kGearRatio;
    private static final double kEncoderUnitsPerDeg = kEncoderUnitsPerRev/360.0;
    public static final int degreesToEncoderUnits(double _degrees) {return (int)(_degrees * kEncoderUnitsPerDeg);}
    public static final double encoderUnitsToDegrees(double _encoderUnits) {return (double)(_encoderUnits / kEncoderUnitsPerDeg);}

    // PID
    private final ProfiledPIDController pid;
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kMaxVelocityDegPerSecond = 60;
    private static final double kMaxAccelerationDegPerSecSquared = 135;

    // Position Check
    private static final double kAtTargetThresholdDegrees = 1.0;

    // Soft Limits
    private static final double kSoftLimitTolerance = 1;
    private static final double kMaxJiggleAmount = 1;

    public enum IntakeState
    {
        DEFENSE     (ArmPosEnum.DEFENSE,        true),
        CALIBRATING (ArmPosEnum.CALIBRATION,    true),
        GROUND      (ArmPosEnum.GROUND,         false),
        GRAB        (ArmPosEnum.GROUND,         true),
        DUMP        (ArmPosEnum.RAISED,         true),
        DROP        (ArmPosEnum.DROP,           false);

        public final ArmPosEnum armPos;
        public final boolean closedClaw;
        IntakeState(ArmPosEnum armPos, boolean closedClaw) {this.armPos = armPos; this.closedClaw = closedClaw;}
    }

    public IntakeState intakeStatus = IntakeState.DEFENSE;

    public enum ArmPosEnum
    {
        DEFENSE     (91),
        CALIBRATION (91),
        GROUND      (0),
        RAISED      (50),
        JIGGLE_DOWN (35),
        DROP        (91);

        public final double angleDeg;
        ArmPosEnum(double angleDeg) {this.angleDeg = angleDeg;}
    }

    public ArmPosEnum armPosition = intakeStatus.armPos;

    private Intake()
    {
        leftArmMotor = new TalonFX(Constants.kLeftArmMotorID);
        rightArmMotor = new TalonFX(Constants.kRightArmMotorID);

        calSwitch = new DigitalInput(Constants.kArmCalSwitchCh);

        leftArmMotor.configFactoryDefault();
        leftArmMotor.setInverted(TalonFXInvertType.Clockwise);
        leftArmMotor.setNeutralMode(NeutralMode.Brake);
        leftArmMotor.configForwardSoftLimitThreshold(degreesToEncoderUnits(ArmPosEnum.DROP.angleDeg + kSoftLimitTolerance));
        leftArmMotor.configReverseSoftLimitThreshold(degreesToEncoderUnits(ArmPosEnum.GROUND.angleDeg - kSoftLimitTolerance));
        
        rightArmMotor.configFactoryDefault();
        rightArmMotor.follow(leftArmMotor);
        rightArmMotor.setNeutralMode(NeutralMode.Brake);
        rightArmMotor.setInverted(TalonFXInvertType.CounterClockwise);

        clawSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.kClawSolenoidFChannel);

        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(kMaxVelocityDegPerSecond, kMaxAccelerationDegPerSecSquared);
        pid = new ProfiledPIDController(kP, kI, kD, constraints);
        pid.reset(ArmPosEnum.CALIBRATION.angleDeg);

        calibrated = false;
    }

    private boolean jiggleStep = false;
    private double jiggleCount = 0;
    @Override
    public void run()
    {
        disabledInit = true;
        leftArmMotor.configForwardSoftLimitEnable(true);
        leftArmMotor.configReverseSoftLimitEnable(true);

        leftArmMotor.setNeutralMode(NeutralMode.Brake);
        rightArmMotor.setNeutralMode(NeutralMode.Brake);

        if(autoCalibrate && !calibrated) {setState(IntakeState.CALIBRATING);}

        switch(intakeStatus)
        {
            case DUMP:
                if(isAtPos(ArmPosEnum.RAISED))
                {
                    jiggleStep = false;
                    if(jiggleStep)
                        jiggleCount++;
                }
                if(isAtPos(ArmPosEnum.JIGGLE_DOWN))
                {
                    jiggleStep = true;
                }
                setTargetPos(jiggleStep ? ArmPosEnum.RAISED : ArmPosEnum.JIGGLE_DOWN);
                if(jiggleCount >= kMaxJiggleAmount)
                    setState(IntakeState.DROP);
            break;
            case DROP:
                if(isAtPos(ArmPosEnum.DROP))
                    setState(IntakeState.GROUND);
            default:
                jiggleCount = 0;
                jiggleStep = true;
                // leftArmMotor.set(ControlMode.PercentOutput, Controls.getInstance().getAxis(JoystickEnum.THRUSTMASTER).y * 0.5);
                // setTargetPos(Controls.getInstance().getButton(ButtonControlEnum.CLAW_GRAB) ? (Controls.getInstance().getButton(ButtonControlEnum.INTAKE_NEXT_STATE) ? ArmPosEnum.DROP : ArmPosEnum.RAISED) : ArmPosEnum.GROUND);
                setTargetPos(intakeStatus.armPos);
            break;
            case CALIBRATING:
                calibrated = false;
                leftArmMotor.configForwardSoftLimitEnable(false);
                pid.reset(encoderUnitsToDegrees(leftArmMotor.getSelectedSensorPosition()));
                pid.setGoal(ArmPosEnum.CALIBRATION.angleDeg);
                leftArmMotor.set(TalonFXControlMode.PercentOutput, kCalibrationPercentOutput);
                if(checkCalSwitch())
                    setState(IntakeState.GRAB);
            break;
        }

        clawSolenoid.set(intakeStatus.closedClaw ^ invertSolenoid);
        if(isAtPos((75 + 15) / 2, Math.abs(75 - 15) / 2))
            clawSolenoid.set(true ^ invertSolenoid);

        if(checkCalSwitch())
        {
            calibrated = true;
            if(!prevCalSwitch)
                leftArmMotor.setSelectedSensorPosition(degreesToEncoderUnits(ArmPosEnum.CALIBRATION.angleDeg));
        }
        prevCalSwitch = checkCalSwitch();
    }
    
    private boolean disabledInit = true;
    private double disabledTime, disabledPos;
    @Override
    public void disable() {
        if(disabledInit)
        {
            disabledTime = Timer.getFPGATimestamp();
            disabledPos = encoderUnitsToDegrees(leftArmMotor.getSelectedSensorPosition());
        }
        if(Timer.getFPGATimestamp() - disabledTime > kDisableCoastTimeThreshold)
        {
            leftArmMotor.setNeutralMode(NeutralMode.Coast);
            rightArmMotor.setNeutralMode(NeutralMode.Coast);
        }
        if(Math.abs(encoderUnitsToDegrees(leftArmMotor.getSelectedSensorPosition()) - disabledPos) > kDisableRecalPosThreshold)
            calibrated = false;
        disabledInit = false;
    }

    private double pidOutput = 0.0;
    public void setTargetPos(ArmPosEnum pos)
    {
        armPosition = pos;
        setPos(armPosition);
    }

    private void setPos(ArmPosEnum pos)
    {
        pid.setGoal(pos.angleDeg);
        double currentAngleDegrees = encoderUnitsToDegrees(leftArmMotor.getSelectedSensorPosition());

        pidOutput = pid.calculate(currentAngleDegrees);
        leftArmMotor.set(TalonFXControlMode.PercentOutput, pidOutput);
    }

    public boolean isAtPos(double pos, double threshold)
    {
        double currentAngleDegrees = encoderUnitsToDegrees(leftArmMotor.getSelectedSensorPosition());

        return (Math.abs(currentAngleDegrees - pos) < threshold);
    }
    public boolean isAtPos(ArmPosEnum pos, double threshold)    {return isAtPos(pos.angleDeg, threshold);}
    public boolean isAtPos(ArmPosEnum pos)                      {return isAtPos(pos, kAtTargetThresholdDegrees);}

    public void setState(IntakeState newState)
    {
        intakeStatus = newState;
    }

    private boolean prevCalSwitch;
    public boolean checkCalSwitch()
    {
        return !calSwitch.get();
    }

    private final ShuffleboardTab tab = Shuffleboard.getTab("Intake");
    private final NetworkTableEntry statusEntry = tab.add("Status", "not updating what").withWidget(BuiltInWidgets.kTextView)         .withPosition(0,0).withSize(2,1).getEntry();
    private final NetworkTableEntry armposEntry = tab.add("Arm position", "not updating what").withWidget(BuiltInWidgets.kTextView)   .withPosition(0,1).getEntry();
    private final NetworkTableEntry calibratedEntry = tab.add("Calibrated", false).withWidget(BuiltInWidgets.kBooleanBox)             .withPosition(1,1).getEntry();
    private final NetworkTableEntry solenoidEntry = tab.add("Solenoid", false).withWidget(BuiltInWidgets.kBooleanBox)             .withPosition(2,0).getEntry();
    private final NetworkTableEntry armDegreeEntry = tab.add("Arm Degree", 0).withWidget(BuiltInWidgets.kTextView)             .withPosition(2,1).getEntry();
    private final NetworkTableEntry jiggleCountEntry = tab.add("Jiggle Count", 0).withWidget(BuiltInWidgets.kTextView)             .withPosition(5,0).getEntry();
    private final NetworkTableEntry jiggleStepEntry = tab.add("Jiggle Step", "notupdating what").withWidget(BuiltInWidgets.kTextView)             .withPosition(5,1).getEntry();

    private final NetworkTableEntry goalEntry = tab.add("PID Goal", 0).withWidget(BuiltInWidgets.kTextView)               .withPosition(6,1).getEntry();
    private final NetworkTableEntry errorEntry = tab.add("PID Error", 0).withWidget(BuiltInWidgets.kTextView)             .withPosition(7,1).getEntry();
    private final NetworkTableEntry outputEntry = tab.add("PID Output", 0).withWidget(BuiltInWidgets.kTextView)           .withPosition(8,1).getEntry();
    private final NetworkTableEntry pEntry = tab.add("PID P", kP).withWidget(BuiltInWidgets.kTextView)                    .withPosition(6,2).getEntry();
    private final NetworkTableEntry iEntry = tab.add("PID I", kI).withWidget(BuiltInWidgets.kTextView)                    .withPosition(7,2).getEntry();
    private final NetworkTableEntry dEntry = tab.add("PID D", kD).withWidget(BuiltInWidgets.kTextView)                    .withPosition(8,2).getEntry();
    private final NetworkTableEntry updateButtonEntry = tab.add("Update PID", false).withWidget(BuiltInWidgets.kToggleButton) .withPosition(6,3).withSize(3,1).getEntry();

    @Override
    public void updateShuffleboard() {
        statusEntry.setString(intakeStatus.name());
        armposEntry.setString(armPosition.name());
        calibratedEntry.setBoolean(calibrated);
        armDegreeEntry.setDouble(encoderUnitsToDegrees(leftArmMotor.getSelectedSensorPosition()));

        jiggleCountEntry.setDouble(jiggleCount);
        jiggleStepEntry.setString((jiggleStep ? ArmPosEnum.RAISED : ArmPosEnum.JIGGLE_DOWN).name());

        solenoidEntry.setBoolean(clawSolenoid.get());

        goalEntry.setDouble(pid.getGoal().position);
        errorEntry.setDouble(pid.getGoal().position - encoderUnitsToDegrees(leftArmMotor.getSelectedSensorPosition()));
        outputEntry.setDouble(pidOutput);
        if(updateButtonEntry.getBoolean(false))
        {
            pid.setPID(pEntry.getDouble(0),
                       iEntry.getDouble(0),
                       dEntry.getDouble(0));
            pid.reset(encoderUnitsToDegrees(leftArmMotor.getSelectedSensorPosition()));
            updateButtonEntry.setBoolean(false);
        }
    }
}
