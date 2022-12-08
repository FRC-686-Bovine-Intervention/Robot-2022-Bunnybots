package frc.robot.auto.modes;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.lib.util.Pose;
import frc.robot.lib.util.Vector2d;

/**
 * Interface that holds all the field measurements 
 */

public class FieldDimensions 
{
	// dimensions of field components
	public static final double kFieldLengthX = 648;       // 54'
	public static final double kFieldLengthY = 324;       // 27'
    
    public static final Vector2d firstCratePos = new Vector2d(39.5, 0); // Distance from auto line to crate minus intake distance from bumper
    public static final double crateWidth = 13;
    
    public static final Vector2d redBall3of3   = new Vector2d(93, -85);   // near center
    public static final Vector2d blueBall3of3   = new Vector2d(94, -85);   // near center
    public static final Vector2d[] ball3of3 = new Vector2d[]{redBall3of3,blueBall3of3};
    public static final Vector2d threeBallAutoFinalTarget  = new Vector2d(51, 127);   // other side of field, left wall
    
}