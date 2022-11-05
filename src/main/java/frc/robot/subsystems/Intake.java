package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;
import frc.robot.controls.Controls;
import frc.robot.controls.Controls.JoystickEnum;

public class Intake extends Subsystem{
    private Intake instance;
    public Intake getInstance() {if(instance == null){instance = new Intake();} return instance;}

    private TalonFX leftArmMotor, rightArmMotor;
    private DoubleSolenoid clawSolenoid;
    private static final boolean invertSolenoid = false;

    private static final double kCalibrationPercentOutput = 0.2;

    private static final double kGearRatio = 0;  // 16 in gearbox, 48t:12t sprockets
    private static final double kEncoderUnitsPerRev = 2048 * kGearRatio;
    private static final double kEncoderUnitsPerDeg = kEncoderUnitsPerRev/360.0;

    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kMaxVelocityDegPerSecond = 90;
    private static final double kMaxAccelerationDegPerSecSquared = 180;

    private static final double kAtTargetThresholdDegrees = 5.0;

    private static final double kDisableRecalTimeThreshold = 5;

    private ProfiledPIDController pid;
    
    public enum ArmPosEnum
    {
        DEFENSE(80),
        CALIBRATION(120),
        GROUND(0),
        RAISED(80),
        DROP(120);

        public final double angleDeg;
        ArmPosEnum(double angleDeg) {this.angleDeg = angleDeg;}
    }

    public ArmPosEnum armPosition = ArmPosEnum.DEFENSE;

    public enum IntakeState
    {
        DEFENSE(ArmPosEnum.DEFENSE, false),
        CALIBRATING(ArmPosEnum.CALIBRATION, false),
        GROUND(ArmPosEnum.GROUND, false),
        GRAB(ArmPosEnum.GROUND, true),
        RAISED(ArmPosEnum.RAISED, true),
        DROP(ArmPosEnum.DROP, false);

        public final ArmPosEnum armPos;
        public final boolean closedClaw;
        IntakeState(ArmPosEnum armPos, boolean closedClaw) {this.armPos = armPos; this.closedClaw = closedClaw;}
    }

    public IntakeState intakeStatus = IntakeState.DEFENSE;

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
        leftArmMotor.configForwardSoftLimitEnable(true);
        leftArmMotor.configReverseSoftLimitEnable(true);
        if(autoCalibrate && !calibrated) {setState(IntakeState.CALIBRATING);}

        switch(intakeStatus)
        {
            default:
                leftArmMotor.set(ControlMode.PercentOutput, Controls.getInstance().getAxis(JoystickEnum.THRUSTMASTER).y * 0.5);
                //setTargetPos(armPosition);
            break;
            case CALIBRATING:
                leftArmMotor.configForwardSoftLimitEnable(false);
                pid.reset(encoderUnitsToDegrees(leftArmMotor.getSelectedSensorPosition()));
                calibrated = false;
                leftArmMotor.set(TalonFXControlMode.PercentOutput, kCalibrationPercentOutput);
            break;
        }

        clawSolenoid.set(intakeStatus.closedClaw ^ invertSolenoid ? Value.kForward : Value.kReverse);

        if (checkFwdLimitSwitch())
        {
            leftArmMotor.setSelectedSensorPosition(degreesToEncoderUnits(ArmPosEnum.CALIBRATION.angleDeg));
            calibrated = true;
            setState(IntakeState.DEFENSE);
            if (!prevFwdLimitSwitchClosed)
            {
                leftArmMotor.set(TalonFXControlMode.PercentOutput, 0);
                pid.reset(ArmPosEnum.CALIBRATION.angleDeg);
                pid.setGoal(ArmPosEnum.CALIBRATION.angleDeg);
            }
            prevFwdLimitSwitchClosed = checkFwdLimitSwitch();
        }
    }

    @Override
    public void updateShuffleboard() {
        // TODO Auto-generated method stub
        
    }


    public static int degreesToEncoderUnits(double _degrees) {return (int)(_degrees * kEncoderUnitsPerDeg);}
    public static double encoderUnitsToDegrees(double _encoderUnits) {return (double)(_encoderUnits / kEncoderUnitsPerDeg);}

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
}
