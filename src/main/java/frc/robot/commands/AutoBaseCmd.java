package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.patched.SwerveControllerCommand;

public class AutoBaseCmd extends SequentialCommandGroup {
    private Trajectory trajA = new Trajectory();
    private Trajectory trajB = new Trajectory();
    public AutoBaseCmd(SwerveSubsystem s_Swerve, ShoulderSubsystem m_shoulder, ExtendArmSubsystem m_arm, GrabberSubsystem m_grabber, String trajAPath, String trajBPath){
        Path trajectoryPathA = Filesystem.getDeployDirectory().toPath().resolve(trajAPath);
        Path trajectoryPathB = Filesystem.getDeployDirectory().toPath().resolve(trajBPath);
        try {
            trajA = TrajectoryUtil.fromPathweaverJson(trajectoryPathA);
            trajB = TrajectoryUtil.fromPathweaverJson(trajectoryPathB);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand1 =
            new SwerveControllerCommand(
                trajA,
                s_Swerve::getPose,
                Constants.DriveConstants.kDriveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
        
        SwerveControllerCommand swerveControllerCommand2 =
        new SwerveControllerCommand(
            trajB,
            s_Swerve::getPose,
            Constants.DriveConstants.kDriveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);


        addCommands(
            new InstantCommand(() -> s_Swerve.resetHeadingAndPose()),
            new InstantCommand(() -> s_Swerve.resetOdometry(trajA.getInitialPose())),
            new InstantCommand(() -> System.out.println(s_Swerve.getPose())),







            
            new InstantCommand(() -> System.out.println(s_Swerve.getHeading())),
            new InstantCommand(() -> System.out.println(trajA.getTotalTimeSeconds())),
            //first place preloaded cone
            /*new ParallelCommandGroup(
                new InstantCommand(() -> m_shoulder.setPos_deg(55.0)),
                new InstantCommand(() -> m_arm.setPos_inch(30.0)),
                new InstantCommand(() -> m_grabber.grabberExtend())
            ),

            //wait 0.5 secs
            new InstantCommand(() -> Timer.delay(0.5)),

            //retract shoulder and arm
            new ParallelCommandGroup(
                new InstantCommand(() -> m_shoulder.setPos_deg(0.0)),
                new InstantCommand(() -> m_arm.setPos_inch(0.0))
            ),*/

            //drive to new cone
            swerveControllerCommand1,
            /*
            //extend shoulder and arm again
            new ParallelCommandGroup(
                new InstantCommand(() -> m_shoulder.setPos_deg(55.0)),
                new InstantCommand(() -> m_arm.setPos_inch(30.0))
            ),

            //close grabber around cone
            new InstantCommand(() -> m_grabber.grabberRetract()),

            //retract shoulder and arm
            new ParallelCommandGroup(
                new InstantCommand(() -> m_shoulder.setPos_deg(0.0)),
                new InstantCommand(() -> m_arm.setPos_inch(0.0))
            ),

            //drive back to grid
            swerveControllerCommand2,

            //extend shoulder and arm
            new ParallelCommandGroup(
                new InstantCommand(() -> m_shoulder.setPos_deg(55.0)),
                new InstantCommand(() -> m_arm.setPos_inch(30.0))
            ),

            //open grabber
            new InstantCommand(() -> m_grabber.grabberExtend()),*/
            //stop swerve!
            new InstantCommand(() -> s_Swerve.stopModules()),
            new InstantCommand(() -> System.out.println(s_Swerve.getPose())),
            new InstantCommand(() -> System.out.println(s_Swerve.getHeading()))
        );
    }
}