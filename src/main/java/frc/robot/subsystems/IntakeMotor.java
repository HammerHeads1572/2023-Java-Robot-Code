package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeMotor extends SubsystemBase{
    
    private CANSparkMax m_intakeMotor;

    public IntakeMotor(int motorID) {

        m_intakeMotor = new CANSparkMax(motorID, MotorType.kBrushless);

    }


}
