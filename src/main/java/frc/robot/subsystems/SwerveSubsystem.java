package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    public boolean isFR = true;
    public final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    public final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    public final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    public final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);
    
    public final SwerveModule[] swerveMods = {frontLeft, frontRight, backLeft, backRight};
    public final AHRS gyro = new AHRS(SerialPort.Port.kUSB);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, Rotation2d.fromDegrees(-gyro.getAngle()), getModulePositions());

    public SwerveSubsystem() {
        new Thread(() -> { // delays navX recalibration by 1s as it will be busy recalibrating, placed on a new thread to prevent interruption
            try {
                Thread.sleep(1000);
                zeroHeading();
            } 
            catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public void switchFR() {
        isFR = !isFR;
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        gyro.reset();
        odometer.resetPosition(Rotation2d.fromDegrees(-gyro.getAngle()), getModulePositions(), pose);
    }

    public void straighten() {
        for (SwerveModule s_mod: swerveMods) {
            s_mod.turningMotor.set(s_mod.turningPidController.calculate(s_mod.getAbsoluteEncoderRad(), 0));
            s_mod.turningMotor.set(0);
        }
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < swerveMods.length; i++) {
            positions[i] = swerveMods[i].getPosition();
        }
        return positions;
    }

    @Override
    public void periodic() {
        odometer.update(Rotation2d.fromDegrees(-gyro.getAngle()), getModulePositions());
        SmartDashboard.putNumber("fl drive", frontLeft.getDrivePosition());
        SmartDashboard.putNumber("fr drive", frontRight.getDrivePosition());
        SmartDashboard.putNumber("bl drive", backLeft.getDrivePosition());
        SmartDashboard.putNumber("br drive", backRight.getDrivePosition());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putNumber("Front Left Encoder", frontLeft.getTurningPosition());
        SmartDashboard.putNumber("Front Right Encoder", frontRight.getTurningPosition());
        SmartDashboard.putNumber("Back Left Encoder", backLeft.getTurningPosition());
        SmartDashboard.putNumber("Back Right Encoder", backRight.getTurningPosition());
    }

    public void stopModules() {
        for (SwerveModule sMod : swerveMods)
            sMod.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond); //scales all speeds down instead of truncating them if over max
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}