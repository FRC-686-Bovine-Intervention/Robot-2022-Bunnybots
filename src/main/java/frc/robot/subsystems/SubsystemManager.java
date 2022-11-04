package frc.robot.subsystems;

import java.util.ArrayList;

public class SubsystemManager {
    private static SubsystemManager instance;
    public static SubsystemManager getInstance() {if(instance == null){instance = new SubsystemManager();}return instance;}

    private SubsystemManager() {}

    public ArrayList<Subsystem> subsystems = new ArrayList<>();

    public void init()
    {
        subsystems.add(Drive.getInstance());
    }

    public void run()                   {for (Subsystem s : subsystems) {if (s.Enabled) s.run();            else s.disable();}}
    public void runTestMode()           {for (Subsystem s : subsystems) {if (s.Enabled) s.runTestMode();    else s.disable();}}
    public void runCalibration()        {for (Subsystem s : subsystems) {if (s.Enabled) s.runCalibration();}}
    public void disable()               {for (Subsystem s : subsystems) {s.disable();}}
    public void updateShuffleboard()    {for (Subsystem s : subsystems) {s.updateShuffleboard();}}
}
