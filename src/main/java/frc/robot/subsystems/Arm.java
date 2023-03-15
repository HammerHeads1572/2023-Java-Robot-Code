package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;

import java.time.Instant;

import frc.robot.Constants;

public class Arm extends SubsystemBase
{
    public static WPI_TalonFX m_ArmDriveMotor;
    private WPI_TalonFX m_FollowMotor;
    private double m_TargetAngle;
    private CANCoder angleEncoder;

    // private Slot0Configs m_Slot0Configs = new Slot0Configs();
    //private double m_Offset;
    //private double m_TicksToRotation = 0.000244140625;
    private double m_DegreesToRotation = 53.45*6;
    
    //Current limit / shutoff
    private boolean m_OverCurrent;
    private Instant m_CurrentBreakTarget;
    private boolean m_Disabled;
    private double m_MaxCurrent;
    public static double arm_angle;

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
        m_ArmDriveMotor = new WPI_TalonFX(leaderID,"Canivore");
        m_ArmDriveMotor.configFactoryDefault();
        m_ArmDriveMotor.configClosedloopRamp(IntakeMotor.rampTime);
        m_ArmDriveMotor.setInverted(Constants.Swerve.angleMotorInvert);
        m_ArmDriveMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        m_FollowMotor = new WPI_TalonFX(followerID,"Canivore");
        m_FollowMotor.setInverted(true);
        
        m_MaxCurrent = 30.0;
        m_CurrentBreakTarget = Instant.now();
        m_Disabled = false;

        m_TargetAngle = 0;
        m_ArmDriveMotor.set(TalonFXControlMode.Position,m_TargetAngle);
        m_FollowMotor.follow(m_ArmDriveMotor);

        //m_Offset = 0;

      

        // change
        
        // from example code
        m_ArmDriveMotor.config_kP(0, kPID[0]);
        m_ArmDriveMotor.config_kI(0, kPID[1]);
        m_ArmDriveMotor.config_kD(0, kPID[2]);

        // Attempt at global current limit
        StatorCurrentLimitConfiguration currentLimitConfig = new StatorCurrentLimitConfiguration
        (true, 30., 30., 0.0);
        m_ArmDriveMotor.configStatorCurrentLimit(currentLimitConfig);
        m_FollowMotor.configStatorCurrentLimit(currentLimitConfig);

    }

    /**
     * Periodic function. Creates new request and uses PID to set to angle?
     */
    @Override
    public void periodic()
    {
        // Create a position closed-loop request

        double current = m_ArmDriveMotor.getStatorCurrent();
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
            m_ArmDriveMotor.set(TalonFXControlMode.Disabled, 1);
        }
        else
        {
            m_ArmDriveMotor.set(TalonFXControlMode.Position,m_TargetAngle);
        }
        m_FollowMotor.follow(m_ArmDriveMotor);
        SmartDashboard.putNumber("arm angle", (m_ArmDriveMotor.getSelectedSensorPosition() / 1024*3.14));

        
    }
    
    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
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
        arm_angle = angle;

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
