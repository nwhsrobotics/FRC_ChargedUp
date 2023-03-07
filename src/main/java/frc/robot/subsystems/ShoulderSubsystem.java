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
    public double m_currentPos_rot = 0.0;  // _rot means in rotation
    public double m_desiredPos_rot = 0.0;
    private static final double MAX_SPEED_ROT_PER_TICK = 1.0; // 1.0 for  least 1.222 seconds 0.5 for least 2.4 seconds (assuming bottlenecking max speed) **NOTE: 1 rotation per tick is equivalent to 1.8 degrees per tick**
    private double m_gearRatio = 200;
    private XboxController xboxController;
    private boolean m_enabled = false;
    private final double MAX_ROT = ((110.0 / 360.0) * m_gearRatio);
    private final double MIN_ROT = ((0.0 / 360.0) * m_gearRatio);
    

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
        m_desiredPos_rot += ((p_degree / 360.0) * m_gearRatio);  //converts degree into rotations and add or subtract specific degree from current degree of shoulder
    }

    public void setPos_deg(double p_degree) {
        m_desiredPos_rot = ((p_degree / 360.0) * m_gearRatio);  //converts degree into rotations and set desired Pos to a specific degree
    }    

    @Override
    public void periodic() {
        if (m_enabled == true) {
            if (m_desiredPos_rot > MAX_ROT) {  //110 degree max
                m_desiredPos_rot = MAX_ROT;
            } else if (m_desiredPos_rot < MIN_ROT) {  //0 degree min
                m_desiredPos_rot = MIN_ROT;
            }

            System.out.println(m_desiredPos_rot);

            double distance_rot = (m_desiredPos_rot - m_currentPos_rot);
            double delta_rot = distance_rot;

            if (delta_rot > MAX_SPEED_ROT_PER_TICK) {  //**NOTE: 1 rotation per tick is equivalent to 1.8 degrees per tick** 1 rot = 1.8 deg
                delta_rot = MAX_SPEED_ROT_PER_TICK;
            } else if (delta_rot < -MAX_SPEED_ROT_PER_TICK) {
                delta_rot = -MAX_SPEED_ROT_PER_TICK;
            }

            m_currentPos_rot += delta_rot;

            m_pidController1.setReference(m_currentPos_rot, ControlType.kPosition);
            m_pidController2.setReference(-m_currentPos_rot, ControlType.kPosition);

            SmartDashboard.putNumber("LeftShoulder Motor1 current", m_shoulderMotor1.getOutputCurrent());
            SmartDashboard.putNumber("RightShoulder Motor2 current", m_shoulderMotor2.getOutputCurrent());

            Logger logger = Logger.getInstance();

            logger.recordOutput("LeftShoulder Motor1 Rotations", m_shoulderRelativeEncoder1.getPosition());
            logger.recordOutput("RightShoulder Motor2 Rotations", m_shoulderRelativeEncoder2.getPosition());
            logger.recordOutput("LeftShoulder Motor1 Degrees", (m_shoulderRelativeEncoder1.getPosition() / m_gearRatio) * 360);
            logger.recordOutput("RightShoulder Motor2 Degrees", (m_shoulderRelativeEncoder2.getPosition() / m_gearRatio) * 360);
        } else {
            return;
        }
    }
}
