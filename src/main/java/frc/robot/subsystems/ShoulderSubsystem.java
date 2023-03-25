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

    private static final double MAX_SPEED_DEG_PER_TICK = 45.0 * SECONDS_PER_TICK; // Move 90 degrees in 2 seconds
    private static final double INITIAL_POS_DEG = -78.0;
    private static final double ARM_IN_LIMIT_DEG = -60.0; // less than this, arm forced in, wrist pitched up.
    private double m_gearRatio = 200;
    private boolean m_enabled = false;
    private final double MAX_DEG = 45.0;
    private final double MIN_DEG = -90.0;
    private Logger logger = Logger.getInstance();
    private ExtendArmSubsystem m_extendArmSubsystem;
    public double m_currentPos_deg; // 0 = arm horizontal, positive = arm up
    public double m_desiredPos_deg;
    private boolean m_positionKnown = false;

    /** Creates a new ShoulderSubsystem. */
    public ShoulderSubsystem(XboxController m_controller) {

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
        m_enabled = ((m_shoulderMotor1 != null) && (m_shoulderMotor2 != null));
    }

    public void changePos_deg(double p_degree) {
        // converts degree into rotations and add or subtract
        // specific degree from current degree of shoulder
        m_desiredPos_deg += p_degree;
        /*
         * if (m_desiredPos_deg < MIN_DEG) {
         * m_desiredPos_deg = MIN_DEG;
         * }
         * 
         * if (m_desiredPos_deg > MAX_DEG) {
         * m_desiredPos_deg = MAX_DEG;
         * }
         */
    }

    public void setPos_deg(double p_degree) {
        // converts degree into rotations and set desired Pos to a specific degree
        m_desiredPos_deg = p_degree;
    }

    public boolean positionKnown() {
        return m_positionKnown;
    }

    @Override
    public void periodic() {
        counter++;
        logger.recordOutput("shoulder.absoluteEncoder", m_shoulderAbsoluteEncoder.getAbsolutePosition());
        if (counter == 150) {
            double absRaw = m_shoulderAbsoluteEncoder.getAbsolutePosition();
            double adjustAbs = absRaw - ShoulderConstants.absOffset;

            if (adjustAbs > 0.5) {
                adjustAbs -= 1.0;
            }
            if (adjustAbs < -0.5) {
                adjustAbs += 1.0;
            }

            adjustAbs *= 360.0;

            logger.recordOutput("shoulder.adjustedAbs", -adjustAbs);
            m_currentPos_deg = -adjustAbs;
            m_desiredPos_deg = -adjustAbs;
            m_shoulderRelativeEncoder1.setPosition(degreesToMotorRotation(m_currentPos_deg));
            m_shoulderRelativeEncoder2.setPosition(degreesToMotorRotation(-m_currentPos_deg));

            m_pidController1.setReference(degreesToMotorRotation(m_currentPos_deg), ControlType.kPosition);
            m_pidController2.setReference(-degreesToMotorRotation(m_currentPos_deg), ControlType.kPosition);

            System.out.printf("===========================Abs raw: %f, adjusted abs: %f\n", absRaw, adjustAbs);
            System.out.printf("===========================Desired pos: %f, Current pos: %f\n\n", m_desiredPos_deg,
                    m_currentPos_deg);
            m_positionKnown = true;
        }
        if (m_enabled == true && /* m_extendArmSubsystem.m_homed && */ counter > 150) {

            // automatically retracts the arm (mechanical limit to not damage the bumper or
            // base) **NOTE** NOT EXACT NUMBERS

            // Calculates the distance between the current and desired positions in degrees
            double distance_deg = (m_desiredPos_deg - m_currentPos_deg); // this is useless, just for reading distance in degrees instead of rotations

            // Calculates the distance between the current and desired positions in rotations

            // Limits the maximum change in rotation per tick
            if (distance_deg > MAX_SPEED_DEG_PER_TICK) { // **NOTE: 1 rotation per tick is equivalent to 1.8 degrees per tick** 1 rot = 1.8 deg
                distance_deg = MAX_SPEED_DEG_PER_TICK;
            } else if (distance_deg < -MAX_SPEED_DEG_PER_TICK) {
                distance_deg = -MAX_SPEED_DEG_PER_TICK;
            }

            double max_arm_allowed = computeMaxArmExtension(m_currentPos_deg);
            double arm_position = m_extendArmSubsystem.getCurrentPos_inch();

            // System.out.printf("arm position = %f, max allowed = %f\n", arm_position, max_arm_allowed);
            /*
             * if(arm_position <= max_arm_allowed) {
             * m_currentPos_deg += distance_deg ;
             * // System.out.println("ARM MOVED");
             * }
             * else{
             * // System.out.println("ARM MOVEMENT DISALLOWED");
             * }
             */

            m_currentPos_deg += distance_deg;
            /*
             * 
             * if(m_currentPos_deg > MAX_DEG) {
             * m_currentPos_deg = MAX_DEG;
             * 
             * }
             * 
             * if(m_currentPos_deg < MIN_DEG) {
             * m_currentPos_deg = MIN_DEG;
             * }
             */

            m_pidController1.setReference(degreesToMotorRotation(m_currentPos_deg), ControlType.kPosition);
            m_pidController2.setReference(-degreesToMotorRotation(m_currentPos_deg), ControlType.kPosition);

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

    public double getCurrentDegrees() {
        return m_currentPos_deg;
    }

    public double computeMaxArmExtension(double degrees) {
        if (degrees > 0) {
            return 100.0;
        }

        if (degrees < ARM_IN_LIMIT_DEG) {
            return 0.0; // arm needs to be retracted
        }
        double shoulder_rad = Math.toRadians(degrees);
        double offset_y = Math.cos(shoulder_rad) * OFFSET_LENGTH_IN;
        double y = SHOULDER_HEIGHT_IN - offset_y;
        double max_ext = y / Math.sin(Math.abs(shoulder_rad)) - ARM_BASE_LENGTH_IN;
        return max_ext;
    }

    public double getMaxArmExtension() {
        double length_1 = computeMaxArmExtension(m_currentPos_deg);
        double length_2 = computeMaxArmExtension(m_desiredPos_deg);
        return 20.0;
        // return Math.min(length_1, length_2);
    }

    public boolean isMoving() {
        return m_currentPos_deg != m_desiredPos_deg;
    }

    public void setExtendArmSubsystem(ExtendArmSubsystem extendArmSubsystem) {
        m_extendArmSubsystem = extendArmSubsystem;

    }

    public double degreesToMotorRotation(double degrees) {
        return ((degrees / 360.0) * m_gearRatio);
    }

    public double getMinPitch_deg() {
        /*
         * if(m_currentPos_deg <= -88.0)
         * {
         * return 0.0;
         * }
         * if(m_currentPos_deg > -88.0 && m_currentPos_deg < ARM_IN_LIMIT_DEG)
         * {
         * return 25.0; //pitch up inside robot
         * }
         */
        return -90.0; // not in front of robot any pitch is fine
    }
}
