package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import com.ctre.phoenixpro.controls.PositionVoltage;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;


public class TeleopArm extends CommandBase {
    public double m_TargetAngle;
    public double m_Offset;
    public double m_TicksToRotation = 0.000244140625;
    public double m_DegreesToRotation = 0.0027777777;
    public Arm m_arm;
    private RobotContainer m_robotContainer;

    /**
    
     * Converts and sets angle
     * 
     * @param angle : angle in degrees
     */
    public TeleopArm(double angle, Arm localArm)
    {
        m_TargetAngle = angle * m_DegreesToRotation;
        m_Offset = 0; // Currently set to zero as placeholder. Consider future removal if unused.

        m_arm = localArm;
    }

    @Override
    public void execute() {
        m_robotContainer = RobotContainer.getInstance();

        m_arm.setArmAngle(m_TargetAngle);
    
        m_arm.periodic();
    }
    

    
}

