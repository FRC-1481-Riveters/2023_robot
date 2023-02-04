package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class ArmSubsystem extends SubsystemBase {

    private TalonSRX m_intakeMotor;
    private TalonSRX m_intakeMotorFollower;

    public ArmSubsystem() {
        m_intakeMotor = new TalonSRX(ArmConstants.ARM_SHOULDER_MOTOR);
        m_intakeMotorFollower = new TalonSRX(ArmConstants.ARM_SHOULDER_MOTOR_FOLLOWER);
    }

    public void setShoulder( double percent )
    {
        m_intakeMotor.set(ControlMode.PercentOutput, percent);
        m_intakeMotorFollower.set(ControlMode.PercentOutput, percent);
    }
}
