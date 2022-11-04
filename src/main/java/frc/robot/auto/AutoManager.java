package frc.robot.auto;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.auto.modes.AutoMode;
import frc.robot.auto.modes.WheelDiameterCalibrationAuto;
import frc.robot.command_status.RobotState;

public class AutoManager {
    private static AutoManager instance;
    public static AutoManager getInstance() {if(instance == null){instance = new AutoManager();}return instance;}

    AutoModeExecuter autoModeExecuter = null;
    public static double autoInitialDelaySec = 0;

    private ShuffleboardTab tab = Shuffleboard.getTab("Autonomous");
    private SendableChooser<AutoMode> AutoModeChooser = new SendableChooser<>();
    private ComplexWidget wig = tab.add("AutoMode", AutoModeChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    private NetworkTableEntry waitBeforeAuto = tab.add("Wait Before Auto (sec)", 0.5).getEntry();    

    private AutoManager(){}

    public void InitChoices()
    {
        AutoModeChooser.addOption("Wheel Calibration Auto", new WheelDiameterCalibrationAuto());
    }

    public void init()
    {
        if (autoModeExecuter != null)
        {
            autoModeExecuter.stop();
        }
        autoModeExecuter = null;

        autoModeExecuter = new AutoModeExecuter();
        AutoMode autoMode = AutoModeChooser.getSelected();
        RobotState.getInstance().reset(autoMode.getInitialPose());
        autoInitialDelaySec = waitBeforeAuto.getDouble(0.0);
        
        autoModeExecuter.setAutoMode(AutoModeChooser.getSelected());
        autoModeExecuter.start();
    }

    public void stop()
    {
        if (autoModeExecuter != null)
        {
            autoModeExecuter.stop();
        }
        autoModeExecuter = null;
    }
}
