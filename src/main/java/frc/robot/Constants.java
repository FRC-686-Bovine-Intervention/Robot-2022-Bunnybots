package frc.robot;

public class Constants {
    private static Constants instance = null;
    public static Constants getInstance() {if(instance == null){instance = new Constants();}return instance;}

    // Hardware Port Definitions
    // public static final int kPCMID =              100;
    // Drivetrain Hardware
    public static final int kPigeonID =         1;
    public static final int kLeftMasterID =     2;
    public static final int kLeftSlaveID =      3;
    public static final int kRightMasterID =    4;
    public static final int kRightSlaveID =     5;
    // Intake Hardware
    public static final int kLeftArmMotorID =   6;
    public static final int kRightArmMotorID =  7;
    public static final int kArmCalSwitchCh =   9;
    public static final int kClawSolenoidFChannel = 4;
    // Hopper Hardware
    public static final int kBlanketMotorID =               8;
    public static final int kLeftFlapSolenoidFChannel =     6;
    public static final int kRightFlapSolenoidFChannel =    5;
    // Control Hardware
    public static final int kThrustmasterPort =   0;
    public static final int kButtonboardPort =    1;

    public static final double kLoopDt = 0.01;
    public static final int kTalonTimeoutMs = 5;

    // Robot Dimensions
    public static final double kCenterToFrontBumper = 34.25 / 2;
    public static final double kCenterToIntake = kCenterToFrontBumper + 2;
}

