package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;


public class IntakeSubsystem extends SubsystemBase {

    private TalonSRX m_intakeMotor;

    public IntakeSubsystem() 
    {
        m_intakeMotor = new TalonSRX(IntakeConstants.INTAKE_MOTOR);
          
        m_intakeMotor.configFactoryDefault();
        m_intakeMotor.setNeutralMode(NeutralMode.Brake);
        m_intakeMotor.configNeutralDeadband(0.10);
        // Configure Talon  SRX output and sensor direction
        m_intakeMotor.setSensorPhase(false);
        // Set peak current
        m_intakeMotor.configPeakCurrentLimit(15);
        m_intakeMotor.configPeakCurrentDuration(200);
        m_intakeMotor.configContinuousCurrentLimit(10);
        m_intakeMotor.enableCurrentLimit(true);
    }

    public void setIntake( double minus_one_to_one )
    {
        double output;
        output = minus_one_to_one;

        m_intakeMotor.set(ControlMode.PercentOutput, output);
    }
}
