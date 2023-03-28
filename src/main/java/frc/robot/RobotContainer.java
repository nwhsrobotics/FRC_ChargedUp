package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoBaseCmd;
import frc.robot.commands.AutoEngageCmd;
import frc.robot.commands.ExtendArmCmd;
import frc.robot.commands.ExtendArmDPad;
import frc.robot.commands.ShoulderCmd;
import frc.robot.commands.ShoulderControl;
import frc.robot.commands.SwerveAuto;
import frc.robot.commands.SwerveJoystickDefaultCmd;
import frc.robot.commands.WristJoystickCmd;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class RobotContainer {
    //intialization of different subsystems and commands
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    //object for presenting selection of options in shuffleboard/ smartdashboard
    SendableChooser<Command> m_autoChooser = new SendableChooser<>();

    public final Joystick m_driver = new Joystick(1);

    //controller buttons intialized
    public final XboxController m_operator = new XboxController(0);
    public final JoystickButton m_joyA = new JoystickButton(m_operator, 1); // button A
    public final JoystickButton m_joyB = new JoystickButton(m_operator, 2); // button B
    public final JoystickButton m_joyX = new JoystickButton(m_operator, 3); // button X
    public final JoystickButton m_joyY = new JoystickButton(m_operator, 4); // button Y
    public final JoystickButton m_joyLB = new JoystickButton(m_operator, 5); // Left bumper
    public final JoystickButton m_joyRB = new JoystickButton(m_operator, 6); // Right bumper
    public final JoystickButton m_joyBK = new JoystickButton(m_operator, 7); // Back Button
    public final JoystickButton m_joyST = new JoystickButton(m_operator, 8); // Start Button

    public final ShoulderSubsystem m_shoulderSubsystem = new ShoulderSubsystem(m_operator);

    public final ExtendArmSubsystem m_extendArmSubsystem = new ExtendArmSubsystem(m_shoulderSubsystem);

    public final ExtendArmCmd m_ExtendArmCmd0 = new ExtendArmCmd(m_extendArmSubsystem, 0);
    public final ExtendArmCmd m_ExtendArmCmd36 = new ExtendArmCmd(m_extendArmSubsystem, 36.0);

    public final WristSubsystem m_wristSubsystem = new WristSubsystem(m_shoulderSubsystem);

    public final GrabberSubsystem m_grabberSubsystem = new GrabberSubsystem();

    public final SwerveAuto autoCmd = new SwerveAuto(swerveSubsystem, m_shoulderSubsystem, m_extendArmSubsystem,m_grabberSubsystem);

    //autonomous paths
    Command blue1_auto = new AutoBaseCmd(swerveSubsystem, m_shoulderSubsystem, m_extendArmSubsystem, m_grabberSubsystem,
            "paths/Blue1_CubeExit.wpilib.json", "paths/Blue1B.wpilib.json");
    Command blue2_auto = new SequentialCommandGroup(
            new AutoBaseCmd(swerveSubsystem, m_shoulderSubsystem, m_extendArmSubsystem, m_grabberSubsystem,
                    "paths/Blue2A.wpilib.json", "paths/Blue2B.wpilib.json"),
            new AutoEngageCmd(swerveSubsystem));
    Command blue3_auto = new AutoBaseCmd(swerveSubsystem, m_shoulderSubsystem, m_extendArmSubsystem, m_grabberSubsystem,
            "paths/Blue3_CubeExit.wpilib.json", "paths/Blue3B.wpilib.json");
    Command red1_auto = new AutoBaseCmd(swerveSubsystem, m_shoulderSubsystem, m_extendArmSubsystem, m_grabberSubsystem,
            "paths/Red1_CubeExit.wpilib.json", "paths/Red1B.wpilib.json");
    Command red2_auto = new AutoBaseCmd(swerveSubsystem, m_shoulderSubsystem, m_extendArmSubsystem, m_grabberSubsystem,
            "paths/Red2A.wpilib.json", "paths/Red2B.wpilib.json");
    Command red3_auto = new AutoBaseCmd(swerveSubsystem, m_shoulderSubsystem, m_extendArmSubsystem, m_grabberSubsystem,
            "paths/Red3_CubeExit.wpilib.json", "paths/Red3B.wpilib.json");
    /*
     * Command bluecharge = new AutoBaseCmd(swerveSubsystem, m_shoulderSubsystem,
     * m_extendArmSubsystem, m_grabberSubsystem,
     * "paths/BlueChargeStation.wpilib.json", "paths/Red3B.wpilib.json");
     */
    Command bluecharge = new SequentialCommandGroup(
            new AutoBaseCmd(swerveSubsystem, m_shoulderSubsystem, m_extendArmSubsystem, m_grabberSubsystem,
                    "paths/BlueChargeStation.wpilib.json", "paths/Blue2B.wpilib.json"),
            new AutoEngageCmd(swerveSubsystem));
    /*
     * Command redcharge = new AutoBaseCmd(swerveSubsystem, m_shoulderSubsystem,
     * m_extendArmSubsystem, m_grabberSubsystem,
     * "paths/RedChargeStation.wpilib.json", "paths/Red3B.wpilib.json");
     */

     //presets for arm
    Command redcharge = new SequentialCommandGroup(
            new AutoBaseCmd(swerveSubsystem, m_shoulderSubsystem, m_extendArmSubsystem, m_grabberSubsystem,
                    "paths/RedChargeStation.wpilib.json", "paths/Blue2B.wpilib.json"),
            new AutoEngageCmd(swerveSubsystem));

    ParallelCommandGroup insideCmd = new ParallelCommandGroup(
            new ShoulderCmd(m_shoulderSubsystem, -90),
            new ExtendArmCmd(m_extendArmSubsystem, 0));

    ParallelCommandGroup groundCmd = new ParallelCommandGroup(
            new ShoulderCmd(m_shoulderSubsystem, -50),
            new ExtendArmCmd(m_extendArmSubsystem, 5));

    ParallelCommandGroup middleCmd = new ParallelCommandGroup(
            new ShoulderCmd(m_shoulderSubsystem, -23),
            new ExtendArmCmd(m_extendArmSubsystem, 10));

    ParallelCommandGroup topCmd = new ParallelCommandGroup(
            new ShoulderCmd(m_shoulderSubsystem, 5),
            new ExtendArmCmd(m_extendArmSubsystem, 15));

    ParallelCommandGroup shelfCmd = new ParallelCommandGroup(
            new ShoulderCmd(m_shoulderSubsystem, 20),
            new ExtendArmCmd(m_extendArmSubsystem, 18));

    public RobotContainer() {
        //choose autonomous paths in shuffleboard
        m_autoChooser.setDefaultOption("Blue1", blue1_auto);
        m_autoChooser.addOption("blue2", blue2_auto);
        m_autoChooser.addOption("blue3", blue3_auto);
        m_autoChooser.addOption("red1", red1_auto);
        m_autoChooser.addOption("red2", red2_auto);
        m_autoChooser.addOption("red3", red3_auto);
        m_autoChooser.addOption("blue charging station", bluecharge);
        m_autoChooser.addOption("red charging station", redcharge);

        SmartDashboard.putData(m_autoChooser);
        swerveSubsystem.setDefaultCommand(new SwerveJoystickDefaultCmd(swerveSubsystem, m_driver));
        m_extendArmSubsystem.setDefaultCommand(new ExtendArmDPad(m_extendArmSubsystem, m_operator));
        m_shoulderSubsystem.setDefaultCommand(new ShoulderControl(m_shoulderSubsystem, m_operator));
        m_wristSubsystem.setDefaultCommand(new WristJoystickCmd(m_wristSubsystem, m_operator));
        m_shoulderSubsystem.setExtendArmSubsystem(m_extendArmSubsystem);
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        //assign the button bindings to different commands
        new JoystickButton(m_driver, 3).onTrue(new InstantCommand(() -> swerveSubsystem.resetHeadingAndPose()));
        new JoystickButton(m_driver, 2).onTrue(new InstantCommand(() -> swerveSubsystem.switchFR()));
        new JoystickButton(m_driver, 4).onTrue(new InstantCommand(() -> swerveSubsystem.resetHeadingAndPose()));
        // new JoystickButton(m_driver, 11).onTrue(new InstantCommand(() -> swerveSubsystem.brake()));
        // new JoystickButton(m_driver, 5).onTrue(new InstantCommand(() -> swerveSubsystem.brake()));
        m_joyB.onTrue(insideCmd);
        m_joyA.onTrue(groundCmd);
        m_joyX.onTrue(middleCmd);
        m_joyY.onTrue(topCmd);
        m_joyST.onTrue(shelfCmd);
        m_joyRB.onTrue(new InstantCommand(() -> m_grabberSubsystem.grabberExtend()));
        m_joyLB.onTrue(new InstantCommand(() -> m_grabberSubsystem.grabberRetract()));
        new JoystickButton(m_operator, 7).onTrue(new InstantCommand(() -> m_extendArmSubsystem.startHoming())); // left start button
    }

    public Command getAutonomousCommand() {
        //returns what autonomous path is chosen in shuffleboard currently
        return m_autoChooser.getSelected();
    }
}
