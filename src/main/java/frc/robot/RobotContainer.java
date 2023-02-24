package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.ExtendedArmControl;
import frc.robot.commands.GrabberExtendCommand;
import frc.robot.commands.GrabberRetractCommand;
import frc.robot.commands.GrabberTurnOffCommand;
import frc.robot.commands.ShoulderControl;
import frc.robot.commands.ShoulderControlAuto;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class RobotContainer {




    public final GrabberSubsystem m_grabberSubsystem = new GrabberSubsystem();
    public final GrabberExtendCommand m_grabberExtendControl = new GrabberExtendCommand(m_grabberSubsystem);
    public final GrabberRetractCommand m_grabberRetractControl = new GrabberRetractCommand(m_grabberSubsystem);
    public final GrabberTurnOffCommand m_grabberTurnOffControl = new GrabberTurnOffCommand(m_grabberSubsystem);

    public final XboxController xboxController = new XboxController(0);
    public final JoystickButton m_joyA = new JoystickButton(xboxController, 1); // button A
    public final JoystickButton m_joyB = new JoystickButton(xboxController, 2); // button B
    public final JoystickButton m_joyX = new JoystickButton(xboxController, 3); // button X
    public final JoystickButton m_joyY = new JoystickButton(xboxController, 4); // button Y
    public final JoystickButton m_joyLB = new JoystickButton(xboxController, 5); // Left bumper
    public final JoystickButton m_joyRB = new JoystickButton(xboxController, 6); // Right bumper
    public final JoystickButton m_joyVB = new JoystickButton(xboxController, 7); // View Button
    public final JoystickButton m_joyMB = new JoystickButton(xboxController, 8); // Menu Button
    public final POVButton m_dPadUp = new POVButton(xboxController, 0);
    public final POVButton m_dPadRight = new POVButton(xboxController, 90);
    public final POVButton m_dPadDown = new POVButton(xboxController, 180);
    public final POVButton m_dPadLeft = new POVButton(xboxController, 270);

    public final ExtendArmSubsystem m_extendArmSubsystem = new ExtendArmSubsystem();
    public final ExtendedArmControl m_extendedArmControl = new ExtendedArmControl(m_extendArmSubsystem, 0.0);
    public final ExtendedArmControl m_extendedpresetlength1 = new ExtendedArmControl(m_extendArmSubsystem, 50.0);
    public final ExtendedArmControl m_extendedpresetlength2 = new ExtendedArmControl(m_extendArmSubsystem, 20.0);
    public final ExtendedArmControl m_extendedpresetlength3 = new ExtendedArmControl(m_extendArmSubsystem, -10.0);

    public final ShoulderSubsystem m_shoulderSubsystem = new ShoulderSubsystem();
    public final ShoulderControl m_shoulderControl = new ShoulderControl(m_shoulderSubsystem,0);
    public final ShoulderControl m_shoulderPreset0deg = new ShoulderControl(m_shoulderSubsystem,0);
    public final ShoulderControl m_shoulderPreset55deg = new ShoulderControl(m_shoulderSubsystem,55);
    public final ShoulderControl m_shoulderPreset110deg = new ShoulderControl(m_shoulderSubsystem,110);
    
    public final ShoulderControlAuto m_shoulderAutoCmd = new ShoulderControlAuto(m_shoulderSubsystem);
    public RobotContainer() 
    {
        configureButtonBindings();
    }

    private void configureButtonBindings() 
    {
        m_joyLB.onTrue(m_grabberRetractControl);
        m_joyRB.onTrue(m_grabberExtendControl);
    }
}
