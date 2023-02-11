package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShoulderConstants;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;


public class ShoulderSubsystem extends SubsystemBase {

    private TalonSRX m_shoulderMotor;
    private TalonSRX m_shoulderMotorFollower;
    private ShuffleboardTab tab;
    GenericEntry kP;
    private GenericEntry kI, kD, kCruise, kAcceleration;
    private GenericEntry nt_shoulder_pos, nt_shoulder_set;
    private double shoulderPosition;

    public ShoulderSubsystem() 
    {
        m_shoulderMotor = new TalonSRX(ShoulderConstants.SHOULDER_MOTOR);
        m_shoulderMotorFollower = new TalonSRX(ShoulderConstants.SHOULDER_MOTOR_FOLLOWER);
        tab = Shuffleboard.getTab("Shoulder");
        kP = tab.add("kP", ShoulderConstants.SHOULDER_MOTOR_KP).getEntry();
        kI = tab.add("kI", ShoulderConstants.SHOULDER_MOTOR_KI).getEntry();
        kD = tab.add("kD", ShoulderConstants.SHOULDER_MOTOR_KD).getEntry();
        kCruise = tab.add("shoulder_speed", ShoulderConstants.SHOULDER_MOTOR_CRUISE).getEntry();
        kAcceleration = tab.add("shoulder_acceleration", ShoulderConstants.SHOULDER_MOTOR_ACCELERATION).getEntry();
        nt_shoulder_pos = tab.add("shoulder_pos",0).getEntry();
        nt_shoulder_set = tab.add("shoulder_set",0).getEntry();
         
        m_shoulderMotor.configFactoryDefault();
        m_shoulderMotor.setNeutralMode(NeutralMode.Brake);
        m_shoulderMotor.configSelectedFeedbackSensor( FeedbackDevice.CTRE_MagEncoder_Absolute, 0, ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotor.configNeutralDeadband(0.10, ShoulderConstants.TALON_TIMEOUT_MS);
        // Configure Talon  SRX output and sensor direction
        m_shoulderMotor.setSensorPhase(false);
        // Set peak current
        m_shoulderMotor.configPeakCurrentLimit(15, ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotor.configPeakCurrentDuration(200, ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotor.configContinuousCurrentLimit(10, ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotor.enableCurrentLimit(true);
        // Set Motion Magic gains in slot0
        m_shoulderMotor.selectProfileSlot(0, 0);
        m_shoulderMotor.config_kF(0, ShoulderConstants.SHOULDER_MOTOR_KF, ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotor.config_kP(0, kP.getDouble(ShoulderConstants.SHOULDER_MOTOR_KP), ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotor.config_kI(0, kI.getDouble(ShoulderConstants.SHOULDER_MOTOR_KI), ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotor.config_kD(0, kD.getDouble(ShoulderConstants.SHOULDER_MOTOR_KD), ShoulderConstants.TALON_TIMEOUT_MS);
        // Set acceleration and cruise velocity
        m_shoulderMotor.configMotionCruiseVelocity(ShoulderConstants.SHOULDER_MOTOR_CRUISE, ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotor.configMotionAcceleration(ShoulderConstants.SHOULDER_MOTOR_ACCELERATION, ShoulderConstants.TALON_TIMEOUT_MS);
        // Set shoulder motion limits
        m_shoulderMotor.configForwardSoftLimitEnable(true);
        m_shoulderMotor.configForwardSoftLimitThreshold(ShoulderConstants.SHOULDER_MOTOR_MAX);
        m_shoulderMotor.configReverseSoftLimitEnable(true);
        m_shoulderMotor.configReverseSoftLimitThreshold(ShoulderConstants.SHOULDER_MOTOR_MIN);

        m_shoulderMotorFollower.configFactoryDefault();
        m_shoulderMotor.setNeutralMode(NeutralMode.Brake);
        m_shoulderMotorFollower.follow(m_shoulderMotor);
        // Configure Talon  SRX output and sensor direction
        m_shoulderMotorFollower.setSensorPhase(false);
        // Set peak current
        m_shoulderMotorFollower.configPeakCurrentLimit(15, ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotorFollower.configPeakCurrentDuration(200, ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotorFollower.configContinuousCurrentLimit(10, ShoulderConstants.TALON_TIMEOUT_MS);
        m_shoulderMotorFollower.enableCurrentLimit(true);
    }

    @Override
    public void periodic() 
    {
        nt_shoulder_pos.setDouble( m_shoulderMotor.getSelectedSensorPosition() );
    }

    public void setShoulder( double minus_one_to_one )
    {
        m_shoulderMotor.set(ControlMode.PercentOutput, minus_one_to_one);
        nt_shoulder_set.setDouble( 0 );
    }
    
    public void setShoulderPosition(double value){
        shoulderPosition = value;
        m_shoulderMotor.set(ControlMode.MotionMagic, value);
        nt_shoulder_set.setDouble( shoulderPosition );
    }

}
