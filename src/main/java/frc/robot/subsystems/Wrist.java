package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Wrist extends SubsystemBase
{
    public static CANSparkMax m_WristMotor;
    private SparkMaxPIDController m_PidController;
    public static RelativeEncoder m_WristEncoder;
    private double m_TargetRotations;
    private double m_AngleToRotations = 0.11111111;
    //private double angle2;

    /**
     * 
     * @param PID[] array containing kP, kI, kD values, in that order
     * @param outputRange double storing min & max value for PID output
     * @param motorID id of motor controlling wrist
     */
    public Wrist(double []PID, double outputRange, int motorID)
    {
        if (PID.length != 3)
        {
            System.err.println("ERROR: invalid arg for Wrist initializatoin. Must contain 3 kPID values.");
            return;
        }
        // Initialize motor
        m_WristMotor = new CANSparkMax(motorID, MotorType.kBrushless);

        // Reset to factory defaults to avoid unexpected behavior
        m_WristMotor.restoreFactoryDefaults();
        m_WristMotor.setClosedLoopRampRate(4.);

        // Construct a PID controller from sparkmax object
        m_PidController = m_WristMotor.getPIDController();
        // Create encode object to store location
        m_WristEncoder = m_WristMotor.getEncoder();

        // Initialize PID values.
        m_PidController.setP(PID[0]);
        m_PidController.setI(PID[1]);
        m_PidController.setD(PID[2]);

        m_PidController.setIZone(0);
        m_PidController.setFF(0);
        m_PidController.setOutputRange(-outputRange, outputRange);

        m_TargetRotations = 0;
    }

    @Override
    public void periodic()
    {
        m_PidController.setReference(m_TargetRotations, CANSparkMax.ControlType.kPosition);

    }

    public void setWristAngle(double angle)
    {
        m_TargetRotations = m_AngleToRotations * angle;
       // angle2 = angle;
    }
}
