package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickDefaultCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    // private final XboxController m_driver;
    private final Joystick m_driver;

    public SwerveJoystickDefaultCmd(SwerveSubsystem swerveSubsystem, Joystick m_driver) {
        this.swerveSubsystem = swerveSubsystem;
        this.m_driver = m_driver;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double speedCoefficient = m_driver.getTrigger() ? 1 : (-m_driver.getRawAxis(3)) * 0.3 + 0.5;

        double xSpeed = Math.abs(-m_driver.getY()) < OIConstants.kXYDeadband ? 0
                : -m_driver.getY() > 0
                        ? (-m_driver.getY() - OIConstants.kXYDeadband) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * speedCoefficient * (1 / (1 - OIConstants.kXYDeadband))
                        : (-m_driver.getY() + OIConstants.kXYDeadband) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * speedCoefficient * (1 / (1 - OIConstants.kXYDeadband));
        double ySpeed = Math.abs(-m_driver.getX()) < OIConstants.kXYDeadband ? 0
                : -m_driver.getX() > 0
                        ? (-m_driver.getX() - OIConstants.kXYDeadband) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * speedCoefficient * (1 / (1 - OIConstants.kXYDeadband))
                        : (-m_driver.getX() + OIConstants.kXYDeadband) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * speedCoefficient * (1 / (1 - OIConstants.kXYDeadband));
        double rotatingSpeed = Math.abs(-m_driver.getTwist()) < OIConstants.kZDeadband ? 0
                : -m_driver.getTwist() > 0
                        ? (-m_driver.getTwist() - OIConstants.kZDeadband) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * speedCoefficient
                        : (-m_driver.getTwist() + OIConstants.kZDeadband) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * speedCoefficient;

        ChassisSpeeds chassisSpeeds = (swerveSubsystem.isFR)
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotatingSpeed, Rotation2d.fromDegrees(swerveSubsystem.getHeading()))
                : ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotatingSpeed, new Rotation2d(0));

        swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
