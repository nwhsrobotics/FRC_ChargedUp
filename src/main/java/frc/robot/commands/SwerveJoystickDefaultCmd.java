package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickDefaultCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Joystick controller;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private final double preciseSpd = Constants.OIConstants.kPreciseSpdMetersPerSecond;

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
        //ultra precise mode
        if (controller.getPOV() != -1) {
            int POV = controller.getPOV();
            ChassisSpeeds chassisSpeeds;
            if (!controller.getTrigger()) {
                if(POV == 0) {
                    chassisSpeeds = new ChassisSpeeds(preciseSpd,0,0);
                }
                else if (POV == 45) {
                    chassisSpeeds = new ChassisSpeeds(preciseSpd, preciseSpd, 0);
                }
                else if (POV == 90) {
                    chassisSpeeds = new ChassisSpeeds(0,preciseSpd,0);
                }
                else if (POV == 135) {
                    chassisSpeeds = new ChassisSpeeds(-preciseSpd, preciseSpd, 0);
                }
                else if (POV == 180) {
                    chassisSpeeds = new ChassisSpeeds(-preciseSpd, 0, 0);
                }
                else if (POV == 225) {
                    chassisSpeeds = new ChassisSpeeds(-preciseSpd, -preciseSpd, 0);
                }
                else if (POV == 270) {
                    chassisSpeeds = new ChassisSpeeds(0, -preciseSpd, 0);
                }
                else {
                    chassisSpeeds = new ChassisSpeeds(preciseSpd, -preciseSpd, 0);
                }
            }
            else {
                if (POV == 45 || POV == 90 || POV == 135) {
                    chassisSpeeds = new ChassisSpeeds(0,0, preciseSpd);
                }
                else if (POV == 225 || POV == 270 || POV == 315) {
                    chassisSpeeds = new ChassisSpeeds(0, 0, -preciseSpd);
                }
                else {
                    chassisSpeeds = new ChassisSpeeds();
                }
            }
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

            swerveSubsystem.setModuleStates(moduleStates);
        }
        //corner swerve!
        //TODO check if these are the right corners
        else if (controller.getRawButton(3) || controller.getRawButton(4) || controller.getRawButton(5) || controller.getRawButton(6)) {
            double turningSpeed = Math.abs(controller.getTwist()) > 0.15 ? controller.getTwist() : 0.0;
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
        //regular swerve
        else {
            // the flap on the bottom sets the speed when trigger is not held; at the bottom speed is 0.2, top 0.8
            double speedCoefficient = controller.getTrigger() ? 1 : (-controller.getRawAxis(3)) * 0.3 + 0.5; 

            double xSpeed = controller.getY();
            double ySpeed = controller.getX();
            double turningSpeed = controller.getTwist();

            // deadband to counter joystick drift (and driver error)
            xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
            ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
            turningSpeed = Math.abs(turningSpeed) > 0.15 ? turningSpeed : 0.0;

            // make the driving smoother with slew rate limiters (RIP REDLINE)
            xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * speedCoefficient;
            ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * speedCoefficient;
            turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * speedCoefficient;
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, turningSpeed, Rotation2d.fromDegrees(0));

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
