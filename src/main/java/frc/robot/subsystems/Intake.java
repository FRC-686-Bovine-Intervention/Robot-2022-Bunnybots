package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.controls.Controls;
import frc.robot.controls.Controls.JoystickEnum;

public class Intake extends Subsystem{
    private static Intake instance;
    public static Intake getInstance() {if(instance == null){instance = new Intake();} return instance;}

    // Hardware
    private final TalonFX leftArmMotor, rightArmMotor;
    private final DoubleSolenoid clawSolenoid;
    private static final boolean invertSolenoid = false;

    // Calibration
    private static final double kCalibrationPercentOutput = 0.1;
    private static final double kDisableRecalTimeThreshold = 5;

    // Unit Conversion
    private static final double kGearRatio = 0;  // 16 in gearbox, 48t:12t sprockets
    private static final double kEncoderUnitsPerRev = 2048 * kGearRatio;
    private static final double kEncoderUnitsPerDeg = kEncoderUnitsPerRev/360.0;
    public static final int degreesToEncoderUnits(double _degrees) {return (int)(_degrees * kEncoderUnitsPerDeg);}
    public static final double encoderUnitsToDegrees(double _encoderUnits) {return (double)(_encoderUnits / kEncoderUnitsPerDeg);}

    // PID
    private final ProfiledPIDController pid;
    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kMaxVelocityDegPerSecond = 90;
    private static final double kMaxAccelerationDegPerSecSquared = 180;

    // Position Check
    private static final double kAtTargetThresholdDegrees = 5.0;

    public enum IntakeState
    {
        DEFENSE     (ArmPosEnum.DEFENSE,        false),
        CALIBRATING (ArmPosEnum.CALIBRATION,    false),
        GROUND      (ArmPosEnum.GROUND,         false),
        GRAB        (ArmPosEnum.GROUND,         true),
        RAISED      (ArmPosEnum.RAISED,         true),
        DROP        (ArmPosEnum.DROP,           false);

        public final ArmPosEnum armPos;
        public final boolean closedClaw;
        IntakeState(ArmPosEnum armPos, boolean closedClaw) {this.armPos = armPos; this.closedClaw = closedClaw;}
    }

    public IntakeState intakeStatus = IntakeState.DEFENSE;

    public enum ArmPosEnum
    {
        DEFENSE     (80),
        CALIBRATION (120),
        GROUND      (0),
        RAISED      (80),
        DROP        (120);

        public final double angleDeg;
        ArmPosEnum(double angleDeg) {this.angleDeg = angleDeg;}
    }

    public ArmPosEnum armPosition = intakeStatus.armPos;

    private Intake()
    {
        leftArmMotor = new TalonFX(Constants.kLeftArmMotorID);
        rightArmMotor = new TalonFX(Constants.kRightArmMotorID);

        leftArmMotor.configFactoryDefault();
        leftArmMotor.setInverted(TalonFXInvertType.Clockwise);
        leftArmMotor.setNeutralMode(NeutralMode.Brake);
        leftArmMotor.configForwardSoftLimitThreshold(degreesToEncoderUnits(ArmPosEnum.DROP.angleDeg));
        leftArmMotor.configReverseSoftLimitThreshold(degreesToEncoderUnits(ArmPosEnum.GROUND.angleDeg));
        
        rightArmMotor.configFactoryDefault();
        rightArmMotor.follow(leftArmMotor);
        rightArmMotor.setNeutralMode(NeutralMode.Brake);
        rightArmMotor.setInverted(TalonFXInvertType.CounterClockwise);

        clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.kClawSolenoidFChannel, Constants.kClawSolenoidRChannel);

        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(kMaxVelocityDegPerSecond, kMaxAccelerationDegPerSecSquared);
        pid = new ProfiledPIDController(kP, kI, kD, constraints);
        pid.reset(ArmPosEnum.CALIBRATION.angleDeg);

        calibrated = false;
    }
    
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
            case DROP:
                if(isAtPos(ArmPosEnum.DROP))
                    setState(IntakeState.GRAB);
            default:
                leftArmMotor.set(ControlMode.PercentOutput, Controls.getInstance().getAxis(JoystickEnum.THRUSTMASTER).y * 0.5);
                //setTargetPos(intakeStatus.armPos);
            break;
            case CALIBRATING:
                calibrated = true;
                setState(IntakeState.GROUND);
                // leftArmMotor.configForwardSoftLimitEnable(false);
                // pid.reset(encoderUnitsToDegrees(leftArmMotor.getSelectedSensorPosition()));
                // calibrated = false;
                // leftArmMotor.set(TalonFXControlMode.PercentOutput, kCalibrationPercentOutput);
                // if(checkFwdLimitSwitch())
                //     setState(IntakeState.GROUND);
            break;
        }

        clawSolenoid.set(intakeStatus.closedClaw ^ invertSolenoid ? Value.kForward : Value.kReverse);

        // if (checkFwdLimitSwitch())
        // {
        //     leftArmMotor.setSelectedSensorPosition(degreesToEncoderUnits(ArmPosEnum.CALIBRATION.angleDeg));
        //     calibrated = true;
        //     if (!prevFwdLimitSwitchClosed)
        //     {
        //         leftArmMotor.set(TalonFXControlMode.PercentOutput, 0);
        //         pid.reset(ArmPosEnum.CALIBRATION.angleDeg);
        //         pid.setGoal(ArmPosEnum.CALIBRATION.angleDeg);
        //     }
        //     prevFwdLimitSwitchClosed = checkFwdLimitSwitch();
        // }
    }
    
    private boolean disabledInit = true;
    private double disabledTime;
    @Override
    public void disable() {
        if(disabledInit) disabledTime = Timer.getFPGATimestamp();
        if(Timer.getFPGATimestamp() - disabledTime > kDisableRecalTimeThreshold)
        {
            calibrated = false;
            leftArmMotor.setNeutralMode(NeutralMode.Coast);
            rightArmMotor.setNeutralMode(NeutralMode.Coast);
        }
        disabledInit = false;
    }

    double pidOutput = 0.0;
    public void setTargetPos(ArmPosEnum pos)
    {
        armPosition = pos;
        setPos(armPosition);
    }
    private void setPos(ArmPosEnum pos)
    {
        pid.setGoal(pos.angleDeg);
        pidOutput = 0.0;
        double currentAngleDegrees = encoderUnitsToDegrees(leftArmMotor.getSelectedSensorPosition());

        pidOutput = pid.calculate(currentAngleDegrees);
        leftArmMotor.set(TalonFXControlMode.PercentOutput, pidOutput);
    }

    public boolean isAtPos(ArmPosEnum pos, double threshold)
    {
        double currentAngleDegrees = encoderUnitsToDegrees(leftArmMotor.getSelectedSensorPosition());
        double targetDegrees = pos.angleDeg;

        return (Math.abs(currentAngleDegrees - targetDegrees) < threshold);
    }

    public boolean isAtPos(ArmPosEnum pos) {return isAtPos(pos, kAtTargetThresholdDegrees);}

    public void setState(IntakeState newState)
    {
        intakeStatus = newState;
    }

    private boolean prevFwdLimitSwitchClosed;
    public boolean checkFwdLimitSwitch()
    {
        boolean fwdLimitSwitchClosed = (leftArmMotor.isFwdLimitSwitchClosed() == 1);
        return fwdLimitSwitchClosed;
    }

    private ShuffleboardTab tab = Shuffleboard.getTab("Intake");
    private NetworkTableEntry statusEntry = tab.add("Status", "not updating what").withWidget(BuiltInWidgets.kTextView)         .withPosition(0,0).withSize(2,1).getEntry();
    private NetworkTableEntry armposEntry = tab.add("Arm position", "not updating what").withWidget(BuiltInWidgets.kTextView)   .withPosition(0,1).getEntry();
    private NetworkTableEntry calibratedEntry = tab.add("Calibrated", false).withWidget(BuiltInWidgets.kBooleanBox)             .withPosition(1,1).getEntry();

    @Override
    public void updateShuffleboard() {
        statusEntry.setString(intakeStatus.name());
        armposEntry.setString(armPosition.name());
        calibratedEntry.setBoolean(calibrated);
    }
}
