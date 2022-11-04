package frc.robot.subsystems;

public abstract class Subsystem {
    /* 
        private static SUBNAME instance; 
        public static SUBNAME getInstance() {if(instance == null){instance = new SUBNAME();}return instance;} 
 
        private SUBNAME() 
        { 
             
        } 
    */ 

    /**Returns true if the subsystem is supposed to be active, otherwise false*/
    public boolean Enabled = true;

    /**Is called every tick the subsystem is supposed to be active*/
    public abstract void run();
    /**Is called every tick the subsystem is supposed to be active in test mode*/
    public void runTestMode() {run();}
    /**Is called every tick the subsystem is supposed to be inactive*/
    public void disable() {}

    /**Returns true if the subsytsem has been calibrated in the past*/
    public boolean calibrated = true;
    public boolean autoCalibrate = true;
    /**Put all calibration code here */
    public void runCalibration() {calibrated = false;}

    /**Put all the variables you want to project to Shuffleboard here*/
    public abstract void updateShuffleboard();
}
