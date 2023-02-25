package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.Constants;

public class TeleopArm extends CommandBase {
    public double m_TargetAngle;
    public double m_Offset;
    public double m_TicksToRotation = 0.000244140625;
    public double m_DegreesToRotation = 0.0027777777;
    /**
    
     * Converts and sets angle
     * 
     * @param angle : angle in degrees
     */
    public TeleopArm(double angle)
    {
        m_TargetAngle = angle * m_DegreesToRotation;
    }
    
}