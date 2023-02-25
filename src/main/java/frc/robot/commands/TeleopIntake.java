/*package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenixpro.controls.PositionVoltage;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm;


public class TeleopIntake extends CommandBase {
    public boolean m_x;

    public Arm m_arm;
    private RobotContainer m_robotContainer;


    public TeleopIntake(boolean old_x, Arm localArm)
    {
        m_x = old_x; 

        m_arm = localArm;
    }

    @Override
    public void execute() {
        m_robotContainer = RobotContainer.getInstance();

        press intake button {
            go in
            set x to 1
        }
        
        if intake button released and x = 1 {
            hold thing still
        }
        
        press spit button {
            set x to 0
            spit
        }
        
    }
    

    
}

*/
