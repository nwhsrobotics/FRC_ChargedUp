//PID is going to be a pain to tune... 

package frc.robot.commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.List;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class SwerveAuto extends CommandBase {
    Trajectory trajectory;

    private final SwerveSubsystem swerveSubsystem;
    public SwerveAuto(SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // generate trajectory
        trajectory = TrajectoryGenerator.generateTrajectory(
                //start at origin
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1, 0),
                        new Translation2d(1, -1)),
                new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                trajectoryConfig);

        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints); //basically regular PID with speed limit (RIP REDLINE)
        thetaController.enableContinuousInput(-Math.PI, Math.PI);


        /*
        public SwerveControllerCommand​(Trajectory trajectory,
            Supplier<Pose2d> pose,
            SwerveDriveKinematics kinematics,
            PIDController xController,
            PIDController yController,
            ProfiledPIDController thetaController,
            Consumer<SwerveModuleState[]> outputModuleStates,
            Subsystem requirements)
        */
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);
        swerveControllerCommand.execute(); 
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.resetOdometry(trajectory.getInitialPose());
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
