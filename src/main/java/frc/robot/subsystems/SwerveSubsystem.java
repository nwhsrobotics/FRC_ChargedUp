package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
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
    public final AHRS m_gyro = new AHRS(SerialPort.Port.kUSB);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, Rotation2d.fromDegrees(getHeading()), getModulePositions());
    public Logger logger = Logger.getInstance();

    public SwerveSubsystem() {
        try
        {
            Thread.sleep(500);
        }
        catch (InterruptedException e)
        {
        }

        m_gyro.zeroYaw();;
    }

    public double getHeading() {
        return Math.IEEEremainder(m_gyro.getAngle(), 360) * -1;
    }

    public void resetHeadingAndPose() {
        m_gyro.zeroYaw();
        resetOdometry(new Pose2d());
    }

    public void switchFR() {
        isFR = !isFR;
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        for (SwerveModule sModule : swerveMods)
            sModule.driveEncoder.setPosition(0);
        odometer.resetPosition(Rotation2d.fromDegrees(getHeading()), getModulePositions(), pose);
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

    public SwerveControllerCommand getTrajCmd(Trajectory traj) {
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        return new SwerveControllerCommand(
            traj,
            this::getPose,
            Constants.DriveConstants.kDriveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            this::setModuleStates,
            this);
    }

    @Override
    public void periodic() {
        odometer.update(Rotation2d.fromDegrees(getHeading()), getModulePositions());
        logger.recordOutput("swerve.steer.front.left.abs", frontLeft.getAbsoluteEncoderRad());
        logger.recordOutput("swerve.steer.front.right.abs", frontRight.getAbsoluteEncoderRad());
        logger.recordOutput("swerve.steer.back.left.abs", backLeft.getAbsoluteEncoderRad());
        logger.recordOutput("swerve.steer.back.right.abs", backRight.getAbsoluteEncoderRad());
        
        logger.recordOutput("swerve.pose", getPose());
        logger.recordOutput("swerve.heading", getHeading());
        
        logger.recordOutput("swerve.drive.front.left.velocity", frontLeft.getDriveVelocity());
        logger.recordOutput("swerve.drive.front.right.velocity", frontRight.getDriveVelocity());
        logger.recordOutput("swerve.drive.back.left.velocity", backLeft.getDriveVelocity());
        logger.recordOutput("swerve.drive.back.right.velocity", backRight.getDriveVelocity());
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

    public void brake() {
        for (SwerveModule sMod : swerveMods)
            sMod.stop();

        frontLeft.turningMotor.set(frontLeft.turningPidController.calculate(frontLeft.getTurningPosition(), Math.PI/4));
        frontRight.turningMotor.set(frontRight.turningPidController.calculate(frontRight.getTurningPosition(), Math.PI/4));
        backLeft.turningMotor.set(backLeft.turningPidController.calculate(backLeft.getTurningPosition(), Math.PI/4));
        backRight.turningMotor.set(backRight.turningPidController.calculate(backLeft.getTurningPosition(), Math.PI/4));
    }
}