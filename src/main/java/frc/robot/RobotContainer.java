package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoBaseCmd;
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
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    SendableChooser<Command> m_autoChooser = new SendableChooser<>();

    public final Joystick m_driver = new Joystick(1);

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

    public final ShoulderCmd m_shoulderCmd0 = new ShoulderCmd(m_shoulderSubsystem, 0);
    public final ShoulderCmd m_shoulderCmd55 = new ShoulderCmd(m_shoulderSubsystem, 55);
    public final ShoulderCmd m_shoulderCmd110 = new ShoulderCmd(m_shoulderSubsystem, 110);

    public final ExtendArmCmd m_ExtendArmCmd0 = new ExtendArmCmd(m_extendArmSubsystem, 0);
    public final ExtendArmCmd m_ExtendArmCmd36 = new ExtendArmCmd(m_extendArmSubsystem, 36.0);

    // How does the code compile without this? TODO?
    public final WristSubsystem m_wristSubsystem = new WristSubsystem(m_shoulderSubsystem);

    public final GrabberSubsystem m_grabberSubsystem = new GrabberSubsystem();

    public final SwerveAuto autoCmd = new SwerveAuto(swerveSubsystem, m_shoulderSubsystem, m_extendArmSubsystem, m_grabberSubsystem);

    Command blue1_auto = new AutoBaseCmd(swerveSubsystem, m_shoulderSubsystem, m_extendArmSubsystem, m_grabberSubsystem,
            "paths/Blue1A.wpilib.json", "paths/Blue1B.wpilib.json");
    Command blue2_auto = new AutoBaseCmd(swerveSubsystem, m_shoulderSubsystem, m_extendArmSubsystem, m_grabberSubsystem,
            "paths/Blue2A.wpilib.json", "paths/Blue2B.wpilib.json");
    Command blue3_auto = new AutoBaseCmd(swerveSubsystem, m_shoulderSubsystem, m_extendArmSubsystem, m_grabberSubsystem,
            "paths/Blue3A.wpilib.json", "paths/Blue3B.wpilib.json");
    Command red1_auto = new AutoBaseCmd(swerveSubsystem, m_shoulderSubsystem, m_extendArmSubsystem, m_grabberSubsystem,
            "paths/Red1A.wpilib.json", "paths/Red1B.wpilib.json");
    Command red2_auto = new AutoBaseCmd(swerveSubsystem, m_shoulderSubsystem, m_extendArmSubsystem, m_grabberSubsystem,
            "paths/Red2A.wpilib.json", "paths/Red2B.wpilib.json");
    Command red3_auto = new AutoBaseCmd(swerveSubsystem, m_shoulderSubsystem, m_extendArmSubsystem, m_grabberSubsystem,
            "paths/Red3A.wpilib.json", "paths/Red3B.wpilib.json");
    Command bluecharge = new AutoBaseCmd(swerveSubsystem, m_shoulderSubsystem, m_extendArmSubsystem, m_grabberSubsystem,
            "paths/BlueChargeStation.wpilib.json", "paths/Red3B.wpilib.json");
    Command redcharge = new AutoBaseCmd(swerveSubsystem, m_shoulderSubsystem, m_extendArmSubsystem, m_grabberSubsystem,
            "paths/RedChargeStation.wpilib.json", "paths/Red3B.wpilib.json");

    ParallelCommandGroup Inside = new ParallelCommandGroup(
            new InstantCommand(() -> m_shoulderSubsystem.setPos_deg(-95)),
            new InstantCommand(() -> m_extendArmSubsystem.setPos_inch(0)));

    ParallelCommandGroup Ground = new ParallelCommandGroup(
            new InstantCommand(() -> m_shoulderSubsystem.setPos_deg(-25)),
            new InstantCommand(() -> m_extendArmSubsystem.setPos_inch(5)));

    ParallelCommandGroup Middle = new ParallelCommandGroup(
            new InstantCommand(() -> m_shoulderSubsystem.setPos_deg(-10)),
            new InstantCommand(() -> m_extendArmSubsystem.setPos_inch(10)));

    ParallelCommandGroup Top = new ParallelCommandGroup(
            new InstantCommand(() -> m_shoulderSubsystem.setPos_deg(14)),
            new InstantCommand(() -> m_extendArmSubsystem.setPos_inch(20)));

    ParallelCommandGroup Shelf = new ParallelCommandGroup(
            new InstantCommand(() -> m_shoulderSubsystem.setPos_deg(20)),
            new InstantCommand(() -> m_extendArmSubsystem.setPos_inch(20)));

    public RobotContainer() {
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
        new JoystickButton(m_driver, 3).onTrue(new InstantCommand(() -> swerveSubsystem.resetHeadingAndPose()));
        new JoystickButton(m_driver, 2).onTrue(new InstantCommand(() -> swerveSubsystem.switchFR()));
        new JoystickButton(m_driver, 4).onTrue(new InstantCommand(() -> swerveSubsystem.resetHeadingAndPose()));
        new JoystickButton(m_driver, 11).onTrue(new InstantCommand(() -> swerveSubsystem.brake()));
        new JoystickButton(m_driver, 5).onTrue(new InstantCommand(() -> swerveSubsystem.brake()));
        m_joyB.whileTrue(Inside);
        m_joyA.whileTrue(Ground);
        m_joyX.whileTrue(Middle);
        m_joyY.whileTrue(Top);
        m_joyST.whileTrue(Shelf);
        m_joyRB.whileTrue(new InstantCommand(() -> m_grabberSubsystem.grabberExtend()));
        m_joyLB.whileTrue(new InstantCommand(() -> m_grabberSubsystem.grabberRetract()));
        new JoystickButton(m_operator, 7).onTrue(new InstantCommand(() -> m_extendArmSubsystem.startHoming())); // left start button

        //incase if schedule doesn't work as intended
        /*new JoystickButton(m_operator, 2).onTrue(new ParallelCommandGroup( // B button retracts
            new InstantCommand(() -> m_shoulderSubsystem.setPos_deg(0)),
            new InstantCommand(() -> m_extendArmSubsystem.setPos_inch(0))));

        new JoystickButton(m_operator, 1).onTrue(new ParallelCommandGroup( // A button is ground
            new InstantCommand(() -> m_shoulderSubsystem.setPos_deg(25)),
            new InstantCommand(() -> m_extendArmSubsystem.setPos_inch(5))));

        new JoystickButton(m_operator, 3).onTrue(new ParallelCommandGroup( // X button is middle
            new InstantCommand(() -> m_shoulderSubsystem.setPos_deg(40)),
            new InstantCommand(() -> m_extendArmSubsystem.setPos_inch(10))));*/
         
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}
