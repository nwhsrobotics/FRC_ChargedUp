package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShoulderConstants;
import edu.wpi.first.wpilibj.XboxController;

public class ShoulderSubsystem extends SubsystemBase {
    private CANSparkMax m_shoulderMotor1 = null;
    public CANSparkMax m_shoulderMotor2 = null;
    private SparkMaxPIDController m_pidController1 = null;
    private SparkMaxPIDController m_pidController2 = null;
    private RelativeEncoder m_shoulderRelativeEncoder1 = null;
    private RelativeEncoder m_shoulderRelativeEncoder2 = null;
    public double m_currentPos_rot = 0.0; // _rot means in rotation
    public double m_desiredPos_rot = 0.0;
    private static final double MAX_SPEED_ROT_PER_TICK = 1.0; // 1.0 for least 1.222 seconds 0.5 for least 2.4 seconds
                                                              // (assuming bottlenecking max speed) **NOTE: 1 rotation
                                                              // per tick is equivalent to 1.8 degrees per tick**
    private double m_gearRatio = 200;
    private boolean m_enabled = false;
    private final double MAX_ROT = ((110.0 / 360.0) * m_gearRatio);
    private final double MIN_ROT = ((0.0 / 360.0) * m_gearRatio);
    private Logger logger = Logger.getInstance();
    private ExtendArmSubsystem m_ExtendArmSubsystem;
    private int m_currentPose;
    public double m_currentPos_deg;
    public double m_desiredPos_deg;

    /** Creates a new ShoulderSubsystem. */
    public ShoulderSubsystem(XboxController m_controller, ExtendArmSubsystem m_ExtendArmSubsystem) {
        this.m_ExtendArmSubsystem = m_ExtendArmSubsystem;

        m_shoulderMotor1 = new CANSparkMax(ShoulderConstants.LeftShoulderCanID, CANSparkMax.MotorType.kBrushless);
        m_shoulderMotor2 = new CANSparkMax(ShoulderConstants.RightShoulderCanID, CANSparkMax.MotorType.kBrushless);

        /*
         * while (m_shoulderMotor1.getOutputCurrent() < currentLimit &&
         * m_shoulderMotor2.getOutputCurrent() < currentLimit) {
         * m_shoulderMotor1.set(-0.2);
         * m_shoulderMotor2.set(0.2);
         * }
         * if (m_shoulderMotor1.getOutputCurrent() >= currentLimit ||
         * m_shoulderMotor2.getOutputCurrent() >= currentLimit) {
         * m_shoulderMotor1.stopMotor();
         * m_shoulderMotor2.stopMotor();
         * }
         */

        if (m_shoulderMotor1 != null) {
            m_shoulderMotor1.setSmartCurrentLimit(25);
            m_pidController1 = m_shoulderMotor1.getPIDController();
            m_shoulderRelativeEncoder1 = m_shoulderMotor1.getEncoder();
            m_shoulderRelativeEncoder1.setPosition(0);

            m_pidController1.setP(0.5);
            m_pidController1.setI(0.0);
            m_pidController1.setD(0.0);
            m_pidController1.setFF(0.0);
            m_pidController1.setIZone(0.0);
            m_pidController1.setIMaxAccum(0, 0);

            m_pidController1.setOutputRange(ShoulderConstants.kMinOutput, ShoulderConstants.kMaxOutput);
            m_pidController1.setReference(0.0, ControlType.kPosition);
            System.out.println("ShoulderMotor1 initialized");
        }

        if (m_shoulderMotor2 != null) {
            m_shoulderMotor2.setSmartCurrentLimit(25);
            m_pidController2 = m_shoulderMotor2.getPIDController();
            m_shoulderRelativeEncoder2 = m_shoulderMotor2.getEncoder();
            m_shoulderRelativeEncoder2.setPosition(0);

            m_pidController2.setP(0.5);
            m_pidController2.setI(0.0);
            m_pidController2.setD(0.0);
            m_pidController2.setFF(0.0);
            m_pidController2.setIZone(0.0);
            m_pidController2.setIMaxAccum(0, 0);

            m_pidController2.setOutputRange(ShoulderConstants.kMinOutput, ShoulderConstants.kMaxOutput);
            m_pidController2.setReference(0.0, ControlType.kPosition);
            System.out.println("ShoulderMotor2 initialized");
        }
        m_enabled = ((m_shoulderMotor1 != null) && (m_shoulderMotor2 != null));
    }

    public void changePos_deg(double p_degree) {
        m_desiredPos_rot += ((p_degree / 360.0) * m_gearRatio); // converts degree into rotations and add or subtract
                                                                // specific degree from current degree of shoulder
        m_desiredPos_deg = p_degree;
    }

    public void setPos_deg(double p_degree) {
        m_desiredPos_rot = ((p_degree / 360.0) * m_gearRatio); // converts degree into rotations and set desired Pos to
                                                               // a specific degree
        m_desiredPos_deg = p_degree;
    }

    @Override
    public void periodic() {
        if (m_enabled == true && m_ExtendArmSubsystem.m_homed) {
            if (m_currentPos_deg < 23 && m_currentPos_deg >= 20 && m_desiredPos_deg < 20 && m_ExtendArmSubsystem.getPos_inch() > 0.0) {
                m_ExtendArmSubsystem.setPos_inch(0.0);
            } else {
                if (m_desiredPos_rot > MAX_ROT) { // 110 degree max
                    m_desiredPos_rot = MAX_ROT; 
                    m_desiredPos_deg = 110;
                } else if (m_desiredPos_rot < MIN_ROT) { // 0 degree min
                    m_desiredPos_rot = MIN_ROT;
                    m_desiredPos_deg = 0;
                }

                // System.out.println(m_desiredPos_rot);
                double distance_deg= (m_desiredPos_deg - m_currentPos_deg); //this is useless, just for reading distance in degrees instead of rotations

                double distance_rot = (m_desiredPos_rot - m_currentPos_rot);
                double delta_rot = distance_rot;

                if (delta_rot > MAX_SPEED_ROT_PER_TICK) { // **NOTE: 1 rotation per tick is equivalent to 1.8 degrees
                                                          // per
                                                          // tick** 1 rot = 1.8 deg
                    delta_rot = MAX_SPEED_ROT_PER_TICK;
                } else if (delta_rot < -MAX_SPEED_ROT_PER_TICK) {
                    delta_rot = -MAX_SPEED_ROT_PER_TICK;
                }

                m_currentPos_rot += delta_rot;

                m_pidController1.setReference(m_currentPos_rot, ControlType.kPosition);
                m_pidController2.setReference(-m_currentPos_rot, ControlType.kPosition);

                m_currentPos_deg = (m_currentPos_rot / m_gearRatio) * 360;

                logger.recordOutput("shoulder.left.current", m_shoulderMotor1.getOutputCurrent());
                logger.recordOutput("shoulder.right.current", m_shoulderMotor2.getOutputCurrent());
                logger.recordOutput("shoulder.left.rotation", m_shoulderRelativeEncoder1.getPosition());
                logger.recordOutput("shoulder.right.rotation", m_shoulderRelativeEncoder2.getPosition());
                logger.recordOutput("shoulder.left.degrees", (m_shoulderRelativeEncoder1.getPosition() / m_gearRatio) * 360);
                logger.recordOutput("shoulder.right.degrees", (m_shoulderRelativeEncoder2.getPosition() / m_gearRatio) * 360);
                logger.recordOutput("shoulder.distance.left", distance_deg);
            }

        } else {
            return;
        }
    }

    public double geDesiredDegrees() {
        return m_desiredPos_deg;
    }

    public double getCurrentDegrees() {
        return m_currentPos_deg;
    }

    public void setCurrentPose(int pose) {
        m_currentPose = pose;
    }

    public boolean isMoving() {
        return m_currentPos_rot != m_desiredPos_rot;
    }
}
