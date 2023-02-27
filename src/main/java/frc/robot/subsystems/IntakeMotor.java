package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import java.time.Instant;
import frc.robot.Constants;

public class IntakeMotor extends SubsystemBase{
    
    private CANSparkMax m_Motor;
    private double m_Speed;
    private boolean m_Peaking;
    private Instant m_TimeToHold;
    private boolean m_IsHolding;


    public IntakeMotor(int motorID)
    {
        // Initialize intake motor
        m_Motor = new CANSparkMax(motorID, MotorType.kBrushless);
        m_Motor.setInverted(true);

        m_IsHolding = false;

        m_Speed = 0;
    }

    /**
     * Sets motor speed to most recent value
     * Also checks for current spikes and readjusts to holding mode
     */
    @Override
    public void periodic()
    {
        m_Motor.set(m_Speed);

        SmartDashboard.putNumber("Intake Current", m_Motor.getOutputCurrent());
        double current = m_Motor.getOutputCurrent();
        if (m_Speed < 0)
        {
            m_IsHolding = false;
        }
        else if (current > Constants.Intake.currentThreshold && !m_Peaking)
        {
            m_Peaking = true;
            m_TimeToHold = Instant.now().plusMillis(Constants.Intake.msToHold);
        }
        else if(m_Peaking)
        {
            if (current <= Constants.Intake.currentThreshold)
            {
                m_Peaking = false;
            }
            else if (Instant.now().isAfter(m_TimeToHold))
            {
                m_Speed = Constants.Intake.holdSpeed;
                m_IsHolding = true;
            }
        }
    }


    /**
     * @param speed double between -1 and 1 containing speed to set motor to
     */
    public void setSpeed(double speed)
    {
        if (speed != 0)
        {
            m_Speed = speed;
            m_IsHolding = false;
        }
        else if (!m_IsHolding)
        {
            m_Speed = 0;
        }
    }

}
