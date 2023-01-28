package frc.robot;
// IMPORTANT: SET UP FOR FALCONS TO DRIVE AND NEOS TO TURN
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
//Most values are from SDS SwerveLib setup for MK4_L2
public final class Constants {
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = 0.10033; // set up for MK4(i)
        public static final double kDriveMotorGearRatio = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0); // (set up for MK4(i) L2)
        public static final double kTurningMotorGearRatio = (15.0 / 32.0) * (10.0 / 60.0); // (set up for MK4 L2)
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = .525; // P constant for turning
        public static final double kITurning = 0.;
    }

    public static final class DriveConstants {
        // left-to-right distance between the drivetrain wheels, should be measured from center to center AND IN METERS
        public static final double kTrackWidth = 0.52; 
        // front-back distance between drivetrain wheels, should be measured from center to center AND IN METERS 
        public static final double kWheelBase = 0.625;
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), //front left
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), //front right
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), //back left
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); //backright

        public static final int kFrontLeftDriveMotorPort = 2;
        public static final int kBackLeftDriveMotorPort = 4;
        public static final int kFrontRightDriveMotorPort = 7;
        public static final int kBackRightDriveMotorPort = 10;

        public static final int kFrontLeftTurningMotorPort = 1;
        public static final int kBackLeftTurningMotorPort = 3;
        public static final int kFrontRightTurningMotorPort = 5;
        public static final int kBackRightTurningMotorPort = 11;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        // redundant, ignore.
        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 23;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 22;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 20;
        public static final int kBackRightDriveAbsoluteEncoderPort = 21;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        //FOR ALL OFFSETS: turn wheels until they become straight, replace with the value of encoderss
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad =  2.66; //2.655320743830045;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 5.24; //5.226272544326379;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.61; //3.749049045592507;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 5.20; //2.0586022173425307;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 6380.0 / 60.0 * (ModuleConstants.kDriveMotorGearRatio) * ModuleConstants.kWheelDiameterMeters * Math.PI; // set up for Falcons to drive
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = kPhysicalMaxSpeedMetersPerSecond / Math.hypot(DriveConstants.kTrackWidth / 2.0, DriveConstants.kWheelBase / 2.0); //adapted from SDS

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;
        // TODO specify units.
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 1.5;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        
        // test and REPLACE all of these!!
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        //creates a TrapezoidProfile to determine setpoints for autonomous
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final double kDeadband = 0.1; //deadband on controller
        public static final int kJoystickPort = 2;
        public static final double kPreciseSpdMetersPerSecond = 0.5;
    }

    public static final class ShoulderConstants {
        public static final double kp = 1.0;
        public static final double ki = 0.0;
        public static final double kd = 0.0;
        public static final int MotorPort20 = 20;
        public static final int MotorPort21 = 21;
}
}
