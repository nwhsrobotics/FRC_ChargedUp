package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShoulderConstants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;

public class ShoulderSubsystem extends SubsystemBase {
    private int counter = 0;
    private CANSparkMax m_shoulderMotor1 = null;
    public CANSparkMax m_shoulderMotor2 = null;
    private SparkMaxPIDController m_pidController1 = null;
    private SparkMaxPIDController m_pidController2 = null;
    private RelativeEncoder m_shoulderRelativeEncoder1 = null;
    private RelativeEncoder m_shoulderRelativeEncoder2 = null;
    private DutyCycleEncoder m_shoulderAbsoluteEncoder;
    private static final double SHOULDER_HEIGHT_IN = 26.0; // for arm extension calcs
    private static final double ARM_BASE_LENGTH_IN = 28.0;
    private static final double OFFSET_LENGTH_IN = 7.0;
    private static final double SECONDS_PER_TICK = 0.02;
    private static final double MAX_SPEED_DEG_PER_TICK = 10.0 * SECONDS_PER_TICK; // Move 90 degrees in 2 seconds
    private static final double INITIAL_POS_DEG = -78.0;
    private static final double ARM_IN_LIMIT_DEG = -30.0; // less than this, arm forced in, wrist pitched up.
    private double m_gearRatio = 200;
    private boolean m_enabled = false;
    private final double MAX_DEG = 45.0;
    private final double MIN_DEG = -90.0;
    private Logger logger = Logger.getInstance();
    private ExtendArmSubsystem m_extendArmSubsystem;
    public double m_currentPos_deg; // 0 = arm horizontal, positive = arm up
    public double m_desiredPos_deg;
    private boolean m_positionKnown = false;

    public ShoulderSubsystem(XboxController m_controller) {
        // intialize the all motors with PID etc.
        // intialize the dutyCycleEncoder (absolute encoder)
        m_shoulderMotor1 = new CANSparkMax(ShoulderConstants.LeftShoulderCanID, CANSparkMax.MotorType.kBrushless);
        m_shoulderMotor2 = new CANSparkMax(ShoulderConstants.RightShoulderCanID, CANSparkMax.MotorType.kBrushless);

        m_currentPos_deg = INITIAL_POS_DEG;
        m_desiredPos_deg = m_currentPos_deg;

        if (m_shoulderMotor1 != null) {
            m_shoulderAbsoluteEncoder = new DutyCycleEncoder(3);
            m_shoulderMotor1.setSmartCurrentLimit(25);
            m_pidController1 = m_shoulderMotor1.getPIDController();
            m_shoulderRelativeEncoder1 = m_shoulderMotor1.getEncoder();
            m_shoulderRelativeEncoder1.setPosition(degreesToMotorRotation(m_currentPos_deg));

            m_pidController1.setP(0.5);
            m_pidController1.setI(0.0);
            m_pidController1.setD(0.0);
            m_pidController1.setFF(0.0);
            m_pidController1.setIZone(0.0);
            m_pidController1.setIMaxAccum(0, 0);

            m_pidController1.setOutputRange(ShoulderConstants.kMinOutput, ShoulderConstants.kMaxOutput);
            m_pidController1.setReference(degreesToMotorRotation(m_currentPos_deg), ControlType.kPosition);
            System.out.println("ShoulderMotor1 initialized");
        }

        if (m_shoulderMotor2 != null) {
            m_shoulderMotor2.setSmartCurrentLimit(25);
            m_pidController2 = m_shoulderMotor2.getPIDController();
            m_shoulderRelativeEncoder2 = m_shoulderMotor2.getEncoder();
            m_shoulderRelativeEncoder2.setPosition(degreesToMotorRotation(-m_currentPos_deg));

            m_pidController2.setP(0.5);
            m_pidController2.setI(0.0);
            m_pidController2.setD(0.0);
            m_pidController2.setFF(0.0);
            m_pidController2.setIZone(0.0);
            m_pidController2.setIMaxAccum(0, 0);

            m_pidController2.setOutputRange(ShoulderConstants.kMinOutput, ShoulderConstants.kMaxOutput);
            m_pidController2.setReference(degreesToMotorRotation(-m_currentPos_deg), ControlType.kPosition);
            System.out.println("ShoulderMotor2 initialized");
        }
        // if motors are intialized then its enabled
        m_enabled = ((m_shoulderMotor1 != null) && (m_shoulderMotor2 != null));
    }

    public void changePos_deg(double p_degree) {
        // This method changes the desired position of the shoulder joint by a given
        // number of degrees
        m_desiredPos_deg += p_degree;         
    }

    public void setPos_deg(double p_degree) {
        // This method sets the desired position of the shoulder joint to a specific value in degrees
        m_desiredPos_deg = p_degree;
    }

    public boolean positionKnown() {
        // This method returns a boolean value indicating whether the current position of the shoulder joint is known
        return m_positionKnown;
    }

    @Override
    public void periodic() {
        counter++;

        // Log the current absolute position of the shoulder motor
        logger.recordOutput("shoulder.absoluteEncoder", m_shoulderAbsoluteEncoder.getAbsolutePosition());

        // Update the desired position of the shoulder if it is counter is 150 (3 seconds)
        if (counter == 150) {
            // Get the raw absolute position of the shoulder motor and adjust it by the offset
            double absRaw = m_shoulderAbsoluteEncoder.getAbsolutePosition();
            double adjustAbs = absRaw - ShoulderConstants.absOffset;

            // Normalize the adjusted absolute position between -0.5 and 0.5 instead of between 0 and 1
            if (adjustAbs > 0.5) {
                adjustAbs -= 1.0;
            }
            if (adjustAbs < -0.5) {
                adjustAbs += 1.0;
            }

            // Convert the normalized absolute position to degrees
            adjustAbs *= 360.0;

            // Log the adjusted absolute position and update the current and desired positions in degrees
            logger.recordOutput("shoulder.adjustedAbs", -adjustAbs);
            m_currentPos_deg = -adjustAbs;
            m_desiredPos_deg = -adjustAbs;

            // Set the relative encoder positions of the shoulder motors
            m_shoulderRelativeEncoder1.setPosition(degreesToMotorRotation(m_currentPos_deg));
            m_shoulderRelativeEncoder2.setPosition(degreesToMotorRotation(-m_currentPos_deg));

            // Set the PID controller references to the current position
            m_pidController1.setReference(degreesToMotorRotation(m_currentPos_deg), ControlType.kPosition);
            m_pidController2.setReference(-degreesToMotorRotation(m_currentPos_deg), ControlType.kPosition);

            // Print debugging information
            System.out.printf("===========================Abs raw: %f, adjusted abs: %f\n", absRaw, adjustAbs);
            System.out.printf("===========================Desired pos: %f, Current pos: %f\n\n", m_desiredPos_deg, m_currentPos_deg);

            // Indicate that the position is known
            m_positionKnown = true;
        }

        // Move the shoulder motor towards the desired position if enabled and homed after 3 seconds
        if (m_enabled == true && /* m_extendArmSubsystem.m_homed && */ counter > 150) {

            // Calculate the distance between the current and desired positions in degrees
            double distance_deg = (m_desiredPos_deg - m_currentPos_deg);

            // Limit the maximum change in degrees per tick
            if (distance_deg > MAX_SPEED_DEG_PER_TICK) {
                distance_deg = MAX_SPEED_DEG_PER_TICK;
            } else if (distance_deg < -MAX_SPEED_DEG_PER_TICK) {
                distance_deg = -MAX_SPEED_DEG_PER_TICK;
            }

            // Calculate the maximum allowed arm extension and the current arm position
            double max_arm_allowed = computeMaxArmExtension(m_currentPos_deg);
            double arm_position = m_extendArmSubsystem.getCurrentPos_inch();

            if(arm_position <= max_arm_allowed) {
                m_currentPos_deg += distance_deg ;
                System.out.println("ARM MOVED");
            } else{
                System.out.println("ARM MOVEMENT DISALLOWED");
            }

            if(m_currentPos_deg > MAX_DEG) {
             m_currentPos_deg = MAX_DEG;
            } else if(m_currentPos_deg < MIN_DEG) {
             m_currentPos_deg = MIN_DEG;
            }

            // Set the PID controller references to the updated current position
            m_pidController1.setReference(degreesToMotorRotation(m_currentPos_deg), ControlType.kPosition);
            m_pidController2.setReference(-degreesToMotorRotation(m_currentPos_deg), ControlType.kPosition);

            // Log various output values
            logger.recordOutput("shoulder.left.current", m_shoulderMotor1.getOutputCurrent());
            logger.recordOutput("shoulder.current_pos_degrees", m_currentPos_deg);
            logger.recordOutput("shoulder.desired_pos_degrees", m_desiredPos_deg);
            logger.recordOutput("shoulder.right.current", m_shoulderMotor2.getOutputCurrent());
            logger.recordOutput("shoulder.left.rotation", m_shoulderRelativeEncoder1.getPosition());
            logger.recordOutput("shoulder.right.rotation", m_shoulderRelativeEncoder2.getPosition());
            logger.recordOutput("shoulder.left.degrees", (m_shoulderRelativeEncoder1.getPosition() / m_gearRatio) * 360);
            logger.recordOutput("shoulder.right.degrees", (m_shoulderRelativeEncoder2.getPosition() / m_gearRatio) * 360);
            logger.recordOutput("shoulder.distance.left", distance_deg);
        }
    }

    public double geDesiredDegrees() {
        return m_desiredPos_deg;
    }

    // return current postion of shoulder in degrees
    public double getCurrentDegrees() {
        return m_currentPos_deg;
    }

    /**
     * Computes the maximum extension of the arm in inches based on the given angle
     * in degrees.
     * @param degrees the angle in degrees
     * @return the maximum allowed extension of the arm in inches
     */
    public double computeMaxArmExtension(double degrees) {
        if (degrees > -10) {
            return 20.0;
        } else if (degrees <= ARM_IN_LIMIT_DEG) {
            return 0.0; // arm needs to be retracted
        } else if (degrees > ARM_IN_LIMIT_DEG && degrees <= -10) {
            double shoulder_rad = Math.toRadians(degrees);
            double offset_y = Math.cos(shoulder_rad) * OFFSET_LENGTH_IN;
            double y = SHOULDER_HEIGHT_IN - offset_y;
            double max_ext = y / Math.sin(Math.abs(shoulder_rad)) - ARM_BASE_LENGTH_IN;
            return max_ext;
        } else {
            return 0.0; // default return statement
        }
    }
    

    // Returns the maximum extension of the arm in inches.
    public double getMaxArmExtension() {
        double length_1 = computeMaxArmExtension(m_currentPos_deg);
        double length_2 = computeMaxArmExtension(m_desiredPos_deg);
        //return 20.0;
        return Math.min(length_1, length_2);
    }

    // return true if the arm is moving, false otherwise
    public boolean isMoving() {
        return m_currentPos_deg != m_desiredPos_deg;
    }

    // Sets the ExtendArmSubsystem
    public void setExtendArmSubsystem(ExtendArmSubsystem extendArmSubsystem) {
        m_extendArmSubsystem = extendArmSubsystem;
    }

    /**
     * Converts degrees to motor rotation.
     * @param degrees the angle in degrees
     * @return the motor rotation value
     */
    public double degreesToMotorRotation(double degrees) {
        return ((degrees / 360.0) * m_gearRatio);
    }

    // Returns the minimum pitch angle for wrist in degrees
    public double getMinPitch_deg() {
        if(m_currentPos_deg <= -83.0)
        {
            return 0.0;
        }
        if(m_currentPos_deg > -83.0 && m_currentPos_deg < ARM_IN_LIMIT_DEG+4)
        {
            return 25.0; //pitch up inside robot
        }
        else 
        {
         // not in front of robot any pitch is fine            
            return -90.0;
        }
    }
}
