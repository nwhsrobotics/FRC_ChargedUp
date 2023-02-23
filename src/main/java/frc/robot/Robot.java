package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


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

        m_robotContainer.m_extendedArmControl.m_position = 0;

    }

    int currentmapping = 0;
    String Mode = "Default";

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {    
        SmartDashboard.putString("Mode", Mode);
        if (m_robotContainer.xboxController.getRawButtonPressed(5)) {
            // Adding 1 to the mapping
            currentmapping += 1;

            if(currentmapping > 3){
                currentmapping = 0;
            }
        }
        if(currentmapping == 0)
        {
            Mode = "Default";
        }
        else if(currentmapping == 1)
        {
            Mode = "ExtendArm";
            System.out.println("ExtendArm Mode");
            m_robotContainer.m_joyA.onTrue(m_robotContainer.m_extendedpresetlength1);
            m_robotContainer.m_joyB.onTrue(m_robotContainer.m_extendedpresetlength2);
            m_robotContainer.m_joyX.onTrue(m_robotContainer.m_extendedpresetlength3);
            if(m_robotContainer.xboxController.getRawButtonPressed(6))// when right bumper is pressed increase the extendArm distance by 5 inches
            {
                m_robotContainer.m_extendedArmControl.m_position += 5;
            }   
        }
        else if(currentmapping == 2)
        {
            Mode = "Grabber";
            SmartDashboard.putString("Mode", "Grabber");
            System.out.println("Grabber Mode");
            m_robotContainer.m_joyA.onTrue(m_robotContainer.m_grabberExtendControl);
            m_robotContainer.m_joyB.onTrue(m_robotContainer.m_grabberRetractControl);
            m_robotContainer.m_joyX.onTrue(m_robotContainer.m_grabberTurnOffControl);                        
        }
        else if(currentmapping == 3)
        {
            Mode = "Shoulder";
            SmartDashboard.putString("Mode", "Shoulder");
            System.out.println("Shoulder Mode");
            m_robotContainer.m_joyA.onTrue(m_robotContainer.m_shoulderPreset0deg);
            m_robotContainer.m_joyB.onTrue(m_robotContainer.m_shoulderPreset55deg);
            m_robotContainer.m_joyX.onTrue(m_robotContainer.m_shoulderPreset110deg);               
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
