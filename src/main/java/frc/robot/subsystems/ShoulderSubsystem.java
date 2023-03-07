package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShoulderConstants;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ShoulderSubsystem extends SubsystemBase {
    private CANSparkMax m_shoulderMotor1 = null;
    public CANSparkMax m_shoulderMotor2 = null;
    private SparkMaxPIDController m_pidController1 = null;
    private SparkMaxPIDController m_pidController2 = null;
    private RelativeEncoder m_shoulderRelativeEncoder1 = null;
    private RelativeEncoder m_shoulderRelativeEncoder2 = null;
    public double m_currentPos_rot = 0.0;
    public double m_desiredPos_rot = 0.0;
    private static final double TICKS_PER_SECOND = 50.0; // Revisit this value!!!
    private static final double TOTAL_DISTANCE = 50.0; // 50 for 1.222 seconds 25 for 2.4 seconds
    private static final double SECONDS_TO_MOVE = 1.0; // Revisit this value!!!
    private static final double SPEED_ROT_PER_TICK = ((TOTAL_DISTANCE)) / (SECONDS_TO_MOVE * TICKS_PER_SECOND);
    private double m_gearRatio = 200;
    private XboxController xboxController;
    private boolean m_enabled = false;

    /** Creates a new ShoulderSubsystem. */
    public ShoulderSubsystem(XboxController m_controller) {
        this.xboxController = m_controller;
        m_shoulderMotor1 = new CANSparkMax(ShoulderConstants.LeftShoulderCanID, CANSparkMax.MotorType.kBrushless);
        
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
            m_pidController1.setIMaxAccum(0,0);

            m_pidController1.setOutputRange(ShoulderConstants.kMinOutput, ShoulderConstants.kMaxOutput);
            m_pidController1.setReference(0.0, ControlType.kPosition);
            System.out.println("ShoulderMotor1 initialized");
        }

        m_shoulderMotor2 = new CANSparkMax(ShoulderConstants.RightShoulderCanID, CANSparkMax.MotorType.kBrushless);

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
            m_pidController2.setIMaxAccum(0,0);

            m_pidController2.setOutputRange(ShoulderConstants.kMinOutput, ShoulderConstants.kMaxOutput);
            m_pidController2.setReference(0.0, ControlType.kPosition);
            System.out.println("ShoulderMotor2 initialized");
        }

        m_enabled = ((m_shoulderMotor1 != null) && (m_shoulderMotor2 != null));
    }

    public void changePos_deg(double p_degree) {

        m_desiredPos_rot += ((p_degree / 360.0) * m_gearRatio);
    }

    public void resetPos() {
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("motor1 current", m_shoulderMotor1.getOutputCurrent());
        SmartDashboard.putNumber("motor 2 current", m_shoulderMotor2.getOutputCurrent());
        //System.out.println(xboxController.getPOV());
        if (m_enabled == true) {
            if (xboxController.getPOV() == 0) {
                System.out.println("up");
                changePos_deg(1);   //take the shoulder up exactly by 5 degrees when UP D-Pad button pressed
            } else if (xboxController.getPOV() == 180) {
                System.out.println("down");
                changePos_deg(-1);   //take the shoulder down exactly by 5 degrees when DOWN D-Pad button pressed
            }

            if (m_desiredPos_rot > ((110.0 / 360.0) * m_gearRatio)) {
                m_desiredPos_rot = ((110.0 / 360.0) * m_gearRatio);
            } else if (m_desiredPos_rot < ((0.0 / 360.0) * m_gearRatio)) {
                m_desiredPos_rot = ((0.0 / 360.0) * m_gearRatio);
            }

            System.out.println(m_desiredPos_rot);

            double distance = (m_desiredPos_rot - m_currentPos_rot);
            double delta = distance;

            if (delta > SPEED_ROT_PER_TICK) {
                delta = SPEED_ROT_PER_TICK;
            } else if (delta < -SPEED_ROT_PER_TICK) {
                delta = -SPEED_ROT_PER_TICK;
            }

            m_currentPos_rot += delta;


            m_pidController1.setReference(m_currentPos_rot, ControlType.kPosition);
            m_pidController2.setReference(-m_currentPos_rot, ControlType.kPosition);

            Logger logger = Logger.getInstance();

            logger.recordOutput("Shoulder 1 Motor Rotations", m_shoulderRelativeEncoder1.getPosition());
            logger.recordOutput("Shoulder 2 Motor Rotations", m_shoulderRelativeEncoder2.getPosition());
            logger.recordOutput("Shouler 1 Degrees", (m_shoulderRelativeEncoder1.getPosition() / m_gearRatio) * 360);
            logger.recordOutput("Shouler 2 Degrees", (m_shoulderRelativeEncoder2.getPosition() / m_gearRatio) * 360);
        } else {
            return;
        }
    }
}
