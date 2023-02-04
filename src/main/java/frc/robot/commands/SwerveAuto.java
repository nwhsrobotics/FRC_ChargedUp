package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class SwerveAuto extends SequentialCommandGroup {
    public SwerveAuto(SwerveSubsystem s_Swerve){
        // An example trajectory to follow.  All units in meters.
        /*
        Trajectory trajectory1 =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            //List.of(new Translation2d(1, 1), new Translation2d(2, -1), new Translation2d(3, 0)),
            List.of(new Translation2d(1, 1), new Translation2d(2, -1), new Translation2d(3, 0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 4, new Rotation2d(Math.PI)),
            Constants.AutoConstants.autoTrajectoryConfig);
        
        Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(new Pose2d(3, 4, new Rotation2d(Math.PI)), 
            List.of(new Translation2d(3, ), 
            new Translation2d(0, 0)),
            new Pose2d(0, 0, new Rotation2d()), 
            Constants.AutoConstants.autoTrajectoryConfig);

        //Trajectory finalTrajectory = trajectory1.concatenate(trajectory2);*/

        var trajectoryOne =
        TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        List.of(new Translation2d(1, 1), new Translation2d(2, -1), new Translation2d(3, 0)),
        new Pose2d(3, 4, Rotation2d.fromDegrees(-180)),
        Constants.AutoConstants.autoTrajectoryConfig);

        var trajectoryTwo =
        TrajectoryGenerator.generateTrajectory(
        new Pose2d(3, 4, Rotation2d.fromDegrees(-180)),
        List.of(new Translation2d(3, 6), new Translation2d(3, 0)),
        new Pose2d(0, 0, Rotation2d.fromDegrees(90)),
        Constants.AutoConstants.autoTrajectoryConfig);

        var finalTrajectory = trajectoryOne.concatenate(trajectoryTwo);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand1 =
            new SwerveControllerCommand(
                trajectoryOne,
                s_Swerve::getPose,
                Constants.DriveConstants.kDriveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
        
        SwerveControllerCommand swerveControllerCommand2 =
        new SwerveControllerCommand(
            trajectoryTwo,
            s_Swerve::getPose,
            Constants.DriveConstants.kDriveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);


        addCommands(
            new InstantCommand(() -> s_Swerve.resetHeadingAndPose()),
            new InstantCommand(() -> s_Swerve.resetOdometry(finalTrajectory.getInitialPose())),
            new InstantCommand(() -> System.out.println(s_Swerve.getPose())),
            new InstantCommand(() -> System.out.println(s_Swerve.getHeading())),
            new InstantCommand(() -> System.out.println(finalTrajectory.getTotalTimeSeconds())),
            swerveControllerCommand1,
            swerveControllerCommand2,
            new InstantCommand(() -> s_Swerve.stopModules()),
            new InstantCommand(() -> System.out.println(s_Swerve.getPose())),
            new InstantCommand(() -> System.out.println(s_Swerve.getHeading()))
        );
    }
}