package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import frc.robot.Robot;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.time.Instant;

public class Arm extends SubsystemBase
{
    private WPI_TalonFX m_DriveMotor;
    private WPI_TalonFX m_FollowMotor;
    private double m_TargetAngle;
    // private Slot0Configs m_Slot0Configs = new Slot0Configs();
    private double m_Offset;
    //private double m_TicksToRotation = 0.000244140625;
    private double m_DegreesToRotation = 53.45*6;
    
    //Current limit / shutoff
    private boolean m_OverCurrent;
    private Instant m_CurrentBreakTarget;
    private boolean m_Disabled;
    private double m_MaxCurrent;

    /**
     * 
     * @param kPID: double array holding values for kP, kI, kD, in that order
     * @param leaderID: ID of the drive motor
     */
    
    public Arm(double []kPID, int leaderID, int followerID) {
        // Verify the length of kPID array
        if (kPID.length != 3)
        {
            System.err.println("ERROR: INVALID KPID LENGTH IN ARM INIT");
            return;
        }
        m_DriveMotor = new WPI_TalonFX(leaderID,"Canivore");
        m_FollowMotor = new WPI_TalonFX(followerID,"Canivore");
        m_FollowMotor.setInverted(true);

        m_DriveMotor.configFactoryDefault();
        m_MaxCurrent = 0.5;
        m_CurrentBreakTarget = Instant.now();
        m_Disabled = false;

        m_TargetAngle = 0;
        m_DriveMotor.set(TalonFXControlMode.Position,m_TargetAngle);
        m_FollowMotor.follow(m_DriveMotor);

        m_Offset = 0;

        /* are these needed for proper motor initialization ?*/
        m_DriveMotor.configFactoryDefault();

        // change
        m_DriveMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        m_DriveMotor.setInverted(Constants.Swerve.angleMotorInvert);
        m_DriveMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        // from example code
        m_DriveMotor.config_kP(0, kPID[0]);
        m_DriveMotor.config_kI(0, kPID[1]);
        m_DriveMotor.config_kD(0, kPID[2]);
        
    }

    /**
     * Periodic function. Creates new request and uses PID to set to angle?
     */
    @Override
    public void periodic()
    {
        // Create a position closed-loop request
          
        double current = m_DriveMotor.getStatorCurrent();
        if (current > m_MaxCurrent && !m_OverCurrent)
        {
            m_OverCurrent = true;
            m_CurrentBreakTarget = Instant.now().plusMillis(500);
        }
        else if(m_OverCurrent)
        {
            if (current <= m_MaxCurrent)
            {
                m_OverCurrent = false;
            }
            else if (Instant.now().isAfter(m_CurrentBreakTarget))
            {
                m_Disabled = true;
            }
        }
        
        if (m_Disabled)
        {
            m_DriveMotor.set(TalonFXControlMode.Disabled, 1);
        }
        else
        {
            m_DriveMotor.set(TalonFXControlMode.Position,m_TargetAngle);
        }
        m_FollowMotor.follow(m_DriveMotor);
    }
    
   
    /**
     * Converts and sets angle
     * 
     * @param angle : angle in degrees
     */
    public void setArmAngle(double angle)
    {
        if (angle == 0)
        {
            m_Disabled = false;
        }
        m_TargetAngle = angle * m_DegreesToRotation;
SmartDashboard.putNumber("arm angle", angle);
    }
    /**
     * THIS IS TO BE CALLED ONLY IF THERE IS A ERROR WITH THE ENCODE ALLIGNMENT
     * MID-MATCH.
     * Set m_Offset to current encoder value to account for encoder missalignment
     * on an untimely power reset.
     * This should be called when the arm is in a zero'd position.
     */
    // TODO: Figure out how to do this. Get encoder position, convert ticks to rotations.
    //public void ResetFailsafe()
    //{
    //    
    //}
}
