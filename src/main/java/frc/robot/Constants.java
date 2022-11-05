package frc.robot;

public class Constants {
    private static Constants instance = null;
    public static Constants getInstance() {if(instance == null){instance = new Constants();}return instance;}

    // Hardware Port Definitions
    // public static final int kPCMID =              100;
    // Drivetrain Hardware
    public static final int kPigeonID =           100;
    public static final int kLeftMasterID =       100;
    public static final int kLeftSlaveID =        100;
    public static final int kRightMasterID =      100;
    public static final int kRightSlaveID =       100;
    // Intake Hardware
    public static final int kLeftArmMotorID =     100;
    public static final int kRightArmMotorID =    100;
    public static final int kClawSolenoidFChannel =     100;
    public static final int kClawSolenoidRChannel =     100;
    // Hopper Hardware
    public static final int kBlanketMotorID =         100;
    public static final int kLeftFlapSolenoidID =     100;
    public static final int kRightFlapSolenoidID =    100;
    // Control Hardware
    public static final int kThrustmasterPort =   0;
    public static final int kButtonboardPort =    1;

    public static final double kLoopDt = 0.01;
    public static final int kTalonTimeoutMs = 5;

    // Robot Dimensions
    // public static final double kCenterToSideBumper = 15.0;
    // public static final double kCenterToFrontBumper = 19.5;
    // public static final double kCenterToIntake = 32.0;
}

