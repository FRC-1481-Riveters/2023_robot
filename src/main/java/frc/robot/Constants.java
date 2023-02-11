package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;


public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = /* FIXME 1 */ 0.5 / 5.8462;
        public static final double kTurningMotorGearRatio = 1 / 18.0;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.2;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(19.5);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(26.5);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // FIXME: patch these motor IDs up to match the Swervie 2022 configuration
        public static final int kFrontLeftDriveMotorPort = 15;
        public static final int kFrontRightDriveMotorPort = 12;
        public static final int kBackLeftDriveMotorPort = 18;
        public static final int kBackRightDriveMotorPort = 21;

        public static final int kFrontLeftTurningMotorPort = 13;
        public static final int kFrontRightTurningMotorPort = 10;
        public static final int kBackLeftTurningMotorPort = 16;
        public static final int kBackRightTurningMotorPort = 19;

        public static final int gyroPort = 60;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = true;

        // CANCoder IDs
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 14;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 11;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 17;
        public static final int kBackRightDriveAbsoluteEncoderPort = 20;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffset  = 137.7;
        public static final double kFrontRightDriveAbsoluteEncoderOffset = -49.8;
        public static final double kBackLeftDriveAbsoluteEncoderOffset   = -38.5;
        public static final double kBackRightDriveAbsoluteEncoderOffset  = 8.8;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 4;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 
                kPhysicalMaxAngularSpeedRadiansPerSecond;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 6;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 6;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = 
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;
        public static final double kPXController = 5;
        public static final double kPYController = 5;
        public static final double kPThetaController = 2.0;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class ShoulderConstants {
        public static final int SHOULDER_MOTOR = 41;
        public static final int SHOULDER_MOTOR_FOLLOWER = 40;
        public static final double SHOULDER_MOTOR_KF = 0.00;
        public static final double SHOULDER_MOTOR_KP = 5.4;
        public static final double SHOULDER_MOTOR_KI = 0; //0.0150;
        public static final double SHOULDER_MOTOR_KD = 0; //0.0135;
        public static final double SHOULDER_MOTOR_CRUISE = 800;
        public static final double SHOULDER_MOTOR_ACCELERATION = 100;
        public static final int SHOULDER_POSITION_SHELF = -1000;   
        public static final int SHOULDER_POSITION_HIGH = -700;   
        public static final int SHOULDER_POSITION_MID = -600;   
        public static final int SHOULDER_POSITION_LOW = 0;
        public static final int SHOULDER_MOTOR_MIN = 1650;
        public static final int SHOULDER_MOTOR_MAX = 2650;
        public static final int TALON_TIMEOUT_MS = 5000;
    }

    public static final class WristConstants {
        public static final int WRIST_MOTOR = 51;
        public static final int WRIST_MOTOR_CANCODER = 52;
        public static final int WRIST_MOTOR_FOLLOWER = 50;
        public static final double WRIST_MOTOR_KF = 0.00;
        public static final double WRIST_MOTOR_KP = 5.4;
        public static final double WRIST_MOTOR_KI = 0; //0.0150;
        public static final double WRIST_MOTOR_KD = 0; //0.0135;
        public static final double WRIST_MOTOR_CRUISE = 800;
        public static final double WRIST_MOTOR_ACCELERATION = 100;
        public static final int WRIST_POSITION_SHELF = -1000;   
        public static final int WRIST_POSITION_HIGH = -700;   
        public static final int WRIST_POSITION_MID = -600;   
        public static final int WRIST_POSITION_LOW = 0;
        public static final int WRIST_MOTOR_MIN = 2149;  // 189 * 4095 / 360 = 2149 counts
        public static final int WRIST_MOTOR_MAX = 3810;  // 335 * 4095 / 360 = 3810 counts

        public static final int TALON_TIMEOUT_MS = 5000;
    }
    
    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR = 58;
    }

    public static final class ExtendConstants {
        public static final int EXTEND_MOTOR = 55;
        public static final int EXTEND_MOTOR_CANCODER = 56;
        public static final double EXTEND_MOTOR_KF = 0.00;
        public static final double EXTEND_MOTOR_KP = 5.4;
        public static final double EXTEND_MOTOR_KI = 0; //0.0150;
        public static final double EXTEND_MOTOR_KD = 0; //0.0135;
        public static final double EXTEND_MOTOR_CRUISE = 800;
        public static final double EXTEND_MOTOR_ACCELERATION = 100;
        public static final int EXTEND_POSITION_SHELF = 27000;   
        public static final int EXTEND_POSITION_HIGH = 27000;   
        public static final int EXTEND_POSITION_MID = 22000;   
        public static final int EXTEND_POSITION_LOW = 8000;
        public static final int EXTEND_MOTOR_MIN = 500;
        public static final int EXTEND_MOTOR_MAX = 27000;
        public static final int TALON_TIMEOUT_MS = 5000;
    }


    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.15;
    }
}
