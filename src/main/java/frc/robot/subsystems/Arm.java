package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
/*import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.hardware.TalonFX;
*/
import com.ctre.phoenix.motorcontrol.can.TalonFX;


import frc.robot.Robot;
import frc.robot.Constants;



import edu.wpi.first.math.controller.ControlAffinePlantInversionFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase
{
    private TalonFX m_DriveMotor;
    private TalonFX m_FollowMotor;
    private double m_TargetAngle;
    // private Slot0Configs m_Slot0Configs = new Slot0Configs();
    private double m_Offset;
    //private double m_TicksToRotation = 0.000244140625;
    private double m_DegreesToRotation = 53;

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
        m_DriveMotor = new TalonFX(leaderID,"Canivore");
        m_FollowMotor = new TalonFX(followerID,"Canivore");
        m_TargetAngle = 0;
        /* 
        m_Slot0Configs.kP = kPID[0];
        m_Slot0Configs.kI = kPID[1];
        m_Slot0Configs.kD = kPID[2];
        */
        // m_DriveMotor.getConfigurator().apply(m_Slot0Configs);
        m_Offset = 0;

        /* are these needed for proper motor initialization ?*/
        m_DriveMotor.configFactoryDefault();

        // change
        m_DriveMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        m_DriveMotor.setInverted(Constants.Swerve.angleMotorInvert);
        m_DriveMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        // from example code
        //Sm_DriveMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
        m_DriveMotor.config_kP(0, 0.01);
        m_DriveMotor.config_kI(0, 0.0);
        m_DriveMotor.config_kD(0, 0.0);

    
        
    }

    /**
     * Periodic function. Creates new request and uses PID to set to angle?
     */
    @Override
    public void periodic()
    {
        // Create a position closed-loop request
        //removd because phoencx pro command
        // PositionVoltage request = new PositionVoltage(0).Slot(0);
     
        // Set position to targetAngle?
        // m_DriveMotor.setControl(request.withPosition(m_TargetAngle));
        //  m_FollowMotor.setControl(request.withPosition(m_TargetAngle));        
       
        m_DriveMotor.set(TalonFXControlMode.Position,m_TargetAngle);

    }

    /**
     * Converts and sets angle
     * 
     * @param angle : angle in degrees
     */
    public void setArmAngle(double angle)
    {
        m_TargetAngle = angle * m_DegreesToRotation;
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
