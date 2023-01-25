package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickDefaultCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Joystick controller;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveJoystickDefaultCmd(SwerveSubsystem swerveSubsystem, Joystick controller) {
        this.swerveSubsystem = swerveSubsystem;
        this.controller = controller;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (controller.getRawButton(3) || controller.getRawButton(4) || controller.getRawButton(5) || controller.getRawButton(6)) {
            double turningSpeed = Math.abs(controller.getTwist()) > OIConstants.kDeadband ? controller.getTwist() : 0.0;
            turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
            SwerveModuleState[] moduleStates;
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, turningSpeed, swerveSubsystem.getRotation2d());

            if (controller.getRawButton(5)) { //front left
                moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds, new Translation2d(DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2));
            }
            else if (controller.getRawButton(6)) { //front right
                moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds, new Translation2d(DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2));
            }
            else if (controller.getRawButton(3)) { //back left
                moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds, new Translation2d(-DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2));
            }
            else { // back right
                moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds, new Translation2d(-DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2));
            }
            swerveSubsystem.setModuleStates(moduleStates);
        }
        else {
            double speedCoefficient = !controller.getTrigger() ? 1 : OIConstants.kSlowdownFactor;

            // Left joystick controls horizontal movement (moving left joystick left and right moves the robot left and right)
            // moving left joystick up and down moves the robot up and down
            // Right joystick controls rotation (moving right joystick left and right rotates the robot left and right)
            double xSpeed = controller.getY(); // pushing up on the joystick means we want the robot to move forward, so y axis is x component
            double ySpeed = controller.getX();
            double turningSpeed = controller.getTwist();

            // deadband to counter joystick drift (and driver error)
            xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
            ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
            turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

            // make the driving smoother with slew rate limiters (RIP REDLINE)
            xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * speedCoefficient;
            ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * speedCoefficient;
            turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * speedCoefficient;
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, turningSpeed, Rotation2d.fromDegrees(swerveSubsystem.gyro.getAngle()));

            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

            swerveSubsystem.setModuleStates(moduleStates);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // when cmd finishes, make modules stop moving
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
