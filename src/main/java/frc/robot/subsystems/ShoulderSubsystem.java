package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShoulderConstants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShoulderSubsystem extends SubsystemBase {
    private CANSparkMax m_shoulderMotor1 = null;
    public CANSparkMax m_shoulderMotor2 = null;
    private SparkMaxPIDController m_pidController1 = null;
    private SparkMaxPIDController m_pidController2 = null;
    private RelativeEncoder m_shoulderRelativeEncoder1 = null;
    private RelativeEncoder m_shoulderRelativeEncoder2 = null;
    public double m_currentPos = 0.0;
    public double m_desiredPos = 0.0;
    private static final double TICKS_PER_SECOND = 50.0; // Revisit this value!!!
    private static final double TOTAL_DISTANCE = 50.0; // 50 for 1.222 seconds 25 for 2.4 seconds
    private static final double SECONDS_TO_MOVE = 1.0; // Revisit this value!!!
    private static final double SPEED_ROT_PER_TICK = ((TOTAL_DISTANCE)) / (SECONDS_TO_MOVE * TICKS_PER_SECOND);
    private double m_gearRatio = 200;
    private XboxController xboxController;
    private final DutyCycleEncoder m_shoulderAbsoluteEncoderA = new DutyCycleEncoder(1); //TBD
    private final DutyCycleEncoder m_shoulderAbsoluteEncoderB = new DutyCycleEncoder(2); //TBD


    private boolean m_enabled = false;

    /** Creates a new ShoulderSubsystem. */
    public ShoulderSubsystem(XboxController m_controller) {
        this.xboxController = m_controller;
        m_shoulderMotor1 = new CANSparkMax(ShoulderConstants.ShoulderCanID20, CANSparkMax.MotorType.kBrushless);
        
        if (m_shoulderMotor1 != null) {
            m_shoulderMotor1.setSmartCurrentLimit(1);
            m_pidController1 = m_shoulderMotor1.getPIDController();
            m_shoulderRelativeEncoder1 = m_shoulderMotor1.getEncoder();
            m_shoulderRelativeEncoder1.setPosition(0);

            m_pidController1.setP(ShoulderConstants.kp);
            m_pidController1.setI(ShoulderConstants.ki);
            m_pidController1.setD(ShoulderConstants.kd);
            m_pidController1.setIZone(ShoulderConstants.kIz);
            m_pidController1.setFF(ShoulderConstants.kFFz);

            m_pidController1.setOutputRange(ShoulderConstants.kMinOutput, ShoulderConstants.kMaxOutput);
            m_pidController1.setReference(0.0, ControlType.kPosition);
            System.out.println("ShoulderMotor1 initialized");
        }

        m_shoulderMotor2 = new CANSparkMax(ShoulderConstants.ShoulderCanID21, CANSparkMax.MotorType.kBrushless);

        if (m_shoulderMotor2 != null) {
            m_shoulderMotor2.setSmartCurrentLimit(1);
            m_pidController2 = m_shoulderMotor2.getPIDController();
            m_shoulderRelativeEncoder2 = m_shoulderMotor2.getEncoder();
            m_shoulderRelativeEncoder2.setPosition(0);

            m_pidController2.setP(ShoulderConstants.kp);
            m_pidController2.setI(ShoulderConstants.ki);
            m_pidController2.setD(ShoulderConstants.kd);
            
            m_pidController2.setIZone(ShoulderConstants.kIz);
            m_pidController2.setFF(ShoulderConstants.kFFz);

            m_pidController2.setOutputRange(ShoulderConstants.kMinOutput, ShoulderConstants.kMaxOutput);
            m_pidController2.setReference(0.0, ControlType.kPosition);
            System.out.println("ShoulderMotor2 initialized");
            m_enabled = true;
        }
    }

    public void setPos(double p_degree) {
        m_desiredPos = ((p_degree / 360) * m_gearRatio);
    }

    @Override
    public void periodic() {
        if (m_enabled == true) {
            if (xboxController.getPOV() == 0) {
                setPos(m_desiredPos + ((5 / 360) * m_gearRatio));   //take the shoulder up exactly by 5 degrees when UP D-Pad button pressed
            } else if (xboxController.getPOV() == 180) {
                setPos(m_desiredPos - ((5 / 360) * m_gearRatio));   //take the shoulder down exactly by 5 degrees when DOWN D-Pad button pressed
            }
            if (m_desiredPos > ((110 / 360) * m_gearRatio)) {
                m_desiredPos = ((110 / 360) * m_gearRatio);
            } else if (m_desiredPos > ((0 / 360) * m_gearRatio)) {
                m_desiredPos = ((0 / 360) * m_gearRatio);
            }

            double distance = (m_desiredPos - m_currentPos);
            double delta = distance;

            if (delta > SPEED_ROT_PER_TICK) {
                delta = SPEED_ROT_PER_TICK;
            } else if (delta < -SPEED_ROT_PER_TICK) {
                delta = -SPEED_ROT_PER_TICK;
            }

            m_currentPos += delta;

            double absoluteShoulderPosA = m_shoulderAbsoluteEncoderA.getAbsolutePosition();
            double absoluteShoulderPosB = m_shoulderAbsoluteEncoderB.getAbsolutePosition();

            

            m_pidController1.setReference(m_currentPos, ControlType.kPosition);
            m_pidController2.setReference(-m_currentPos, ControlType.kPosition);

            SmartDashboard.putNumber("Shoulder 1 Rotations", m_shoulderRelativeEncoder1.getPosition());
            SmartDashboard.putNumber("Shoulder 2 Rotations", m_shoulderRelativeEncoder2.getPosition());
            SmartDashboard.putNumber("Shouler 1 Inches", (m_shoulderRelativeEncoder1.getPosition() / m_gearRatio) * 360);
            SmartDashboard.putNumber("Shouler 2 Inches", (m_shoulderRelativeEncoder2.getPosition() / m_gearRatio) * 360);
        } else {
            return;
        }
    }
}
