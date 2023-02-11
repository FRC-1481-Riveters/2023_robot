package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExtendConstants;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;


public class ExtendSubsystem extends SubsystemBase {

    private TalonSRX m_extendMotor;
    private CANCoder m_cancoder;
    private ShuffleboardTab tab;
    GenericEntry kP;
    private GenericEntry kI, kD, kCruise, kAcceleration;
    private GenericEntry nt_extend_pos, nt_extend_set;
    private double extendPosition;

    public ExtendSubsystem() 
    {
        m_extendMotor = new TalonSRX(ExtendConstants.EXTEND_MOTOR);
        m_cancoder = new CANCoder(ExtendConstants.EXTEND_MOTOR_CANCODER);
        tab = Shuffleboard.getTab("Extend");
        kP = tab.add("kP", ExtendConstants.EXTEND_MOTOR_KP).getEntry();
        kI = tab.add("kI", ExtendConstants.EXTEND_MOTOR_KI).getEntry();
        kD = tab.add("kD", ExtendConstants.EXTEND_MOTOR_KD).getEntry();
        kCruise = tab.add("extend_speed", ExtendConstants.EXTEND_MOTOR_CRUISE).getEntry();
        kAcceleration = tab.add("extend_acceleration", ExtendConstants.EXTEND_MOTOR_ACCELERATION).getEntry();
        nt_extend_pos = tab.add("extend_pos",0).getEntry();
        nt_extend_set = tab.add("extend_set",0).getEntry();
         
        m_extendMotor.configFactoryDefault();
        m_extendMotor.setNeutralMode(NeutralMode.Brake);
        m_extendMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0);
        m_extendMotor.configRemoteFeedbackFilter(m_cancoder, 0);
        m_extendMotor.configNeutralDeadband(0.10, ExtendConstants.TALON_TIMEOUT_MS);
        // Configure Talon  SRX output and sensor direction
        m_extendMotor.setSensorPhase(false);
        // Set peak current
        m_extendMotor.configPeakCurrentLimit(15, ExtendConstants.TALON_TIMEOUT_MS);
        m_extendMotor.configPeakCurrentDuration(200, ExtendConstants.TALON_TIMEOUT_MS);
        m_extendMotor.configContinuousCurrentLimit(10, ExtendConstants.TALON_TIMEOUT_MS);
        m_extendMotor.enableCurrentLimit(true);
        // Set Motion Magic gains in slot0
        m_extendMotor.selectProfileSlot(0, 0);
        m_extendMotor.config_kF(0, ExtendConstants.EXTEND_MOTOR_KF, ExtendConstants.TALON_TIMEOUT_MS);
        m_extendMotor.config_kP(0, kP.getDouble(ExtendConstants.EXTEND_MOTOR_KP), ExtendConstants.TALON_TIMEOUT_MS);
        m_extendMotor.config_kI(0, kI.getDouble(ExtendConstants.EXTEND_MOTOR_KI), ExtendConstants.TALON_TIMEOUT_MS);
        m_extendMotor.config_kD(0, kD.getDouble(ExtendConstants.EXTEND_MOTOR_KD), ExtendConstants.TALON_TIMEOUT_MS);
        // Set acceleration and cruise velocity
        m_extendMotor.configMotionCruiseVelocity(ExtendConstants.EXTEND_MOTOR_CRUISE, ExtendConstants.TALON_TIMEOUT_MS);
        m_extendMotor.configMotionAcceleration(ExtendConstants.EXTEND_MOTOR_ACCELERATION, ExtendConstants.TALON_TIMEOUT_MS);
        // Set extend motion limits
        m_extendMotor.configForwardSoftLimitEnable(true);
        m_extendMotor.configForwardSoftLimitThreshold(ExtendConstants.EXTEND_MOTOR_MAX);
        m_extendMotor.configReverseSoftLimitEnable(true);
        m_extendMotor.configReverseSoftLimitThreshold(ExtendConstants.EXTEND_MOTOR_MIN);

    }

    @Override
    public void periodic() 
    {
        nt_extend_pos.setDouble( m_extendMotor.getSelectedSensorPosition() );
    }

    public void setExtend( double minus_one_to_one )
    {
        m_extendMotor.set(ControlMode.PercentOutput, minus_one_to_one);
        nt_extend_set.setDouble( 0 );
    }
    
    public void setExtendPosition(double value){
        extendPosition = value;
        m_extendMotor.set(ControlMode.MotionMagic, value);
        nt_extend_set.setDouble( extendPosition );
    }

}
