package frc.robot;

public class Constants {
    private static Constants instance = null;
    public static Constants getInstance() {if(instance == null){instance = new Constants();}return instance;}

    // Hardware Port Definitions
    //public static int kPCMID =              100;
    // Drivetrain Hardware
    public static int kPigeonID =           100;
    public static int kLeftMasterID =       100;
    public static int kLeftSlaveID =        100;
    public static int kRightMasterID =      100;
    public static int kRightSlaveID =       100;
    // Intake Hardware
    public static int kLeftArmMotorID =     100;
    public static int kRightArmMotorID =    100;
    public static int kClawSolenoidFChannel =     100;
    public static int kClawSolenoidRChannel =     100;
    // Hopper Hardware
    public static int kBlanketMotorID =         100;
    public static int kLeftFlapSolenoidID =     100;
    public static int kRightFlapSolenoidID =    100;
    // Control Hardware
    public static int kThrustmasterPort =   0;
    public static int kButtonboardPort =    1;

    public static double kLoopDt = 0.01;
    public static int kTalonTimeoutMs = 5;


    // Robot Dimensions
    public static double kCenterToSideBumper = 15.0;
    public static double kCenterToFrontBumper = 19.5;
    public static double kCenterToIntake = 32.0;
}

