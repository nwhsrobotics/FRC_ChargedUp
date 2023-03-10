package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoBaseCmd;
import frc.robot.commands.ExtendArmCmd;
import frc.robot.commands.ExtendArmDPad;
import frc.robot.commands.ShoulderCmd;
import frc.robot.commands.ShoulderControl;
import frc.robot.commands.SwerveAuto;
import frc.robot.commands.SwerveJoystickDefaultCmd;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class RobotContainer {
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    SendableChooser<Command> m_autoChooser = new SendableChooser<>();

    public final XboxController m_driver = new XboxController(3);
    public final XboxController m_operator = new XboxController(2);
    public final JoystickButton m_joyA = new JoystickButton(m_operator, 1); //button A
    public final JoystickButton m_joyBK = new JoystickButton(m_operator, 7); // Back Button
    public final JoystickButton m_joyST = new JoystickButton(m_operator, 8); // Start Button
    public final JoystickButton m_joyLB = new JoystickButton(m_operator, 5); // Left bumper
    public final JoystickButton m_joyRB = new JoystickButton(m_operator, 6); // Right bumper

    public final ShoulderSubsystem m_shoulderSubsystem = new ShoulderSubsystem(m_operator);
    public final ShoulderCmd m_shoulderCmd0 = new ShoulderCmd(m_shoulderSubsystem, 0);
    public final ShoulderCmd m_shoulderCmd55 = new ShoulderCmd(m_shoulderSubsystem, 55);
    public final ShoulderCmd m_shoulderCmd110 = new ShoulderCmd(m_shoulderSubsystem, 110);

    public final ExtendArmSubsystem m_extendArmSubsystem = new ExtendArmSubsystem();
    public final ExtendArmCmd m_ExtendArmCmd0 = new ExtendArmCmd(m_extendArmSubsystem, 0);
    public final ExtendArmCmd m_ExtendArmCmd36 = new ExtendArmCmd(m_extendArmSubsystem, 36.0);

    public final WristSubsystem m_wristSubsystem = new WristSubsystem(m_operator);

    public final GrabberSubsystem m_grabberSubsystem = new GrabberSubsystem();

    public final SwerveAuto autoCmd = new SwerveAuto(swerveSubsystem, m_shoulderSubsystem, m_extendArmSubsystem, m_grabberSubsystem);

    Command blue1_auto = new AutoBaseCmd(swerveSubsystem, m_shoulderSubsystem, m_extendArmSubsystem, m_grabberSubsystem, "paths/Blue1A.wpilib.json", "paths/Blue1B.wpilib.json");
    Command blue2_auto = new AutoBaseCmd(swerveSubsystem, m_shoulderSubsystem, m_extendArmSubsystem, m_grabberSubsystem, "paths/Blue2A.wpilib.json", "paths/Blue2B.wpilib.json");
    Command blue3_auto = new AutoBaseCmd(swerveSubsystem, m_shoulderSubsystem, m_extendArmSubsystem, m_grabberSubsystem, "paths/Blue3A.wpilib.json", "paths/Blue3B.wpilib.json");
    Command red1_auto = new AutoBaseCmd(swerveSubsystem, m_shoulderSubsystem, m_extendArmSubsystem, m_grabberSubsystem, "paths/Red1A.wpilib.json", "paths/Red1B.wpilib.json");
    Command red2_auto = new AutoBaseCmd(swerveSubsystem, m_shoulderSubsystem, m_extendArmSubsystem, m_grabberSubsystem, "paths/Red2A.wpilib.json", "paths/Red2B.wpilib.json");
    Command red3_auto = new AutoBaseCmd(swerveSubsystem, m_shoulderSubsystem, m_extendArmSubsystem, m_grabberSubsystem, "paths/Red3A.wpilib.json", "paths/Red3B.wpilib.json");




    public RobotContainer() {
        m_autoChooser.setDefaultOption("Blue1", blue1_auto);
        m_autoChooser.addOption("blue2", blue2_auto);
        m_autoChooser.addOption("blue3", blue3_auto);
        m_autoChooser.addOption("red1", red1_auto);
        m_autoChooser.addOption("red2", red2_auto);
        m_autoChooser.addOption("red3", red3_auto);

        SmartDashboard.putData(m_autoChooser);
        swerveSubsystem.setDefaultCommand(new SwerveJoystickDefaultCmd(swerveSubsystem, m_driver));
        m_extendArmSubsystem.setDefaultCommand(new ExtendArmDPad(m_extendArmSubsystem, m_operator));
        m_shoulderSubsystem.setDefaultCommand(new ShoulderControl(m_shoulderSubsystem, m_operator));
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new JoystickButton(m_driver, 8).onTrue(new InstantCommand(() -> swerveSubsystem.resetHeadingAndPose())); //menu
        new JoystickButton(m_driver, 1).onTrue(new InstantCommand(() -> swerveSubsystem.switchFR())); //A
        new JoystickButton(m_driver, 7).onTrue(new InstantCommand(() -> swerveSubsystem.resetHeadingAndPose())); //start
        new JoystickButton(m_operator, 2).onTrue(m_ExtendArmCmd36);
        new JoystickButton(m_operator, 3).onTrue(m_ExtendArmCmd0);
        m_joyRB.whileTrue(new InstantCommand(() -> m_grabberSubsystem.grabberExtend()));
        m_joyLB.whileTrue(new InstantCommand(() -> m_grabberSubsystem.grabberRetract()));
        
        new JoystickButton(m_operator, 1).onTrue(m_shoulderCmd0);
        new JoystickButton(m_operator, 4).onTrue(m_shoulderCmd55);
        new JoystickButton(m_operator, 3).onTrue(m_ExtendArmCmd0);
        new JoystickButton(m_operator, 2).onTrue(m_ExtendArmCmd36);
        new JoystickButton(m_operator, 8).onTrue(new InstantCommand(() -> m_extendArmSubsystem.homing()));
        new JoystickButton(m_operator, 7).onTrue(new InstantCommand(() -> m_extendArmSubsystem.homing()));



    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}
