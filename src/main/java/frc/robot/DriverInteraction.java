package frc.robot; 
 
import frc.robot.controls.Controls;
import frc.robot.subsystems.Drive; 
 
public class DriverInteraction { 
    private static DriverInteraction instance; 
    public static DriverInteraction getInstance() {if(instance == null){instance = new DriverInteraction();}return instance;} 
 
    Drive drive; 
    Controls controls; 
 
    private DriverInteraction() 
    { 
        controls = Controls.getInstance(); 
        drive = Drive.getInstance(); 
    } 
 
    public void init() {
    }
 
    public void run() 
    {
        drive.setOpenLoop(controls.getDriveCommand()); 
    } 
} 
