package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ExtendedArmControl;
import frc.robot.commands.GrabberExtendCommand;
import frc.robot.commands.GrabberRetractCommand;
import frc.robot.commands.GrabberTurnOffCommand;
import frc.robot.commands.ShoulderControl;
import frc.robot.subsystems.ExtendArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

import org.littletonrobotics.junction.LoggedRobot;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

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

    int currentmapping = 0;
    private Command m_autonomousCommand;
    public RobotContainer m_robotContainer;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        // m_robotContainer.swerveSubsystem.resetHeadingAndPose();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }


    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        if (xboxController.getRawButtonPressed(5)) {
            // Switch to mapping 1
            currentmapping = currentmapping + 1;

            if(currentmapping > 3){
                currentmapping = 0;
            }
        }

        if(currentmapping == 1)
        {
            System.out.println("Mode 1");
        }
        else if(currentmapping == 2)
        {
            System.out.println("Mode 2");            
        }

        /*switch (currentmapping) {
            case 1:
            //m_joyA.onTrue(m_extendedpresetlength1);
            System.out.println("Mode 1");
            //m_joyB.onTrue(m_extendedpresetlength2);
            //m_joyX.onTrue(m_extendedpresetlength3);
              break;
            case 2:
            System.out.println("Mode 2");
            //m_joyA.onTrue(m_grabberExtendControl);
            //m_joyB.onTrue(m_grabberRetractControl);
            //m_joyX.onTrue(m_grabberTurnOffControl);
              break;
            case 3:
            System.out.println("Mode 3");
              //Wrist control code will go here when finished
              break;
            default:
            System.out.println("Default");
            //m_joyA.onTrue(m_shoulderPreset0deg);
            //m_joyB.onTrue(m_shoulderPreset55deg);
            //m_joyX.onTrue(m_shoulderPreset110deg);
              break;
             
        } */



        // System.out.println(m_robotContainer.swerveSubsystem.getPose());
    }

    @Override
    public void testInit() {


        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }
}
