package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
public class SwerveJoystickCornerCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final XboxController controller;
    private final SlewRateLimiter turningLimiter;

    public SwerveJoystickCornerCmd(SwerveSubsystem swerveSubsystem, XboxController controller) {
        this.swerveSubsystem = swerveSubsystem;
        this.controller = controller;
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        System.out.println("corner swerve");

        double turningSpeed = controller.getRightX();

        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        SwerveModuleState[] moduleStates;
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, turningSpeed, swerveSubsystem.getRotation2d());
        //rotate around front left
        if (controller.getLeftX() < 0 && controller.getLeftY() > 0) {
            moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds, new Translation2d(DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2));
        }
        //rotate around front right
        else if (controller.getLeftX() > 0 && controller.getLeftY() > 0) {
            moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds, new Translation2d(DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2));
        }
        //rotate around back left
        else if (controller.getLeftX() < 0 && controller.getLeftY() < 0) {
            moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds, new Translation2d(-DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2));
        }
        //rotate around back right
        else {
            moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds, new Translation2d(-DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2));
        }

        swerveSubsystem.setModuleStates(moduleStates);

    }

    @Override
    public void end(boolean interrupted) {
        // when cmd finishes, make modules stop moving or they'll keep moving
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
