package frc.robot.commands;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickDefaultCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final XboxController controller;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveJoystickDefaultCmd(SwerveSubsystem swerveSubsystem, XboxController controller) {
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
        System.out.println(
            swerveSubsystem.frontLeft.getAbsoluteEncoderRad() + " " +
            swerveSubsystem.frontRight.getAbsoluteEncoderRad() + " " +
            swerveSubsystem.backLeft.getAbsoluteEncoderRad() + " " +
            swerveSubsystem.backRight.getAbsoluteEncoderRad()
        );
        if (!swerveSubsystem.isTank) {
            //System.out.println("default swerve");
            // joystick inputs

            // Left joystick controls horizontal movement (moving left joystick left and right moves the robot left and right)
            // moving left joystick up and down moves the robot up and down
            // Right joystick controls rotation (moving right joystick left and right rotates the robot left and right)
            double xSpeed = -controller.getLeftY(); // pushing up on the joystick means we want the robot to move forward, so y axis is x component
            double ySpeed = controller.getLeftX();
            double turningSpeed = controller.getRightX();

            // deadband to counter joystick drift (and driver error)
            xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
            ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
            turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

            // make the driving smoother with slew rate limiters (RIP REDLINE)
            xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
            System.out.println(xSpeed + " " + ySpeed + " " + turningSpeed);
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());

            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

            swerveSubsystem.setModuleStates(moduleStates);
        }
        else {
            System.out.println("tank");
            /* 
            PSEUDO TANK DRIVE, because who wouldn't appreciate a swerve drive robot that drives like a tank drive robot?
            Right Trigger Forward
            Left Trigger Backward
            Left Joystick's X axis to turn
            (yes controls are setup like Redline)
            */
           // System.out.println("psuedo-tank");

            double turningSpeed = controller.getLeftX() * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
            double xSpeed = (-controller.getLeftTriggerAxis() + controller.getRightTriggerAxis()) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            //System.out.println(xSpeed);
            xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
            turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

            //WATCHOUT!!! no limiters on psuedo tank... we don't want a repeat of REDLINE
            //System.out.println(xSpeed);
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, 0.0, turningSpeed); // may have to inverse turningSpeed when reversing (xSpeed < 0)
            //NOTE: pseudo tank is NOT field relative
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

            swerveSubsystem.setModuleStates(moduleStates);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // when cmd finishes, make modules stop moving
        Constants.DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad = swerveSubsystem.frontLeft.getAbsoluteEncoderRad();
        Constants.DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad = swerveSubsystem.backLeft.getAbsoluteEncoderRad();
        Constants.DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad = swerveSubsystem.frontRight.getAbsoluteEncoderRad();
        Constants.DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad = swerveSubsystem.backRight.getAbsoluteEncoderRad();
        try {
            FileWriter fileWriter = new FileWriter("/home/lvuser/offsets.txt");
            BufferedWriter bufferedWriter = new BufferedWriter(fileWriter);
            bufferedWriter.write(String.valueOf(swerveSubsystem.frontLeft.getAbsoluteEncoderRad()));
            bufferedWriter.newLine();
            bufferedWriter.write(String.valueOf(swerveSubsystem.backLeft.getAbsoluteEncoderRad()));
            bufferedWriter.newLine();
            bufferedWriter.write(String.valueOf(swerveSubsystem.frontRight.getAbsoluteEncoderRad()));
            bufferedWriter.newLine();
            bufferedWriter.write(String.valueOf(swerveSubsystem.backRight.getAbsoluteEncoderRad()));
            bufferedWriter.flush();
            bufferedWriter.close();
         } catch (IOException e) {
            e.printStackTrace();
         }
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
