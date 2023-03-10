package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExtendArmConstants;

public class ExtendArmSubsystem extends SubsystemBase {
  public CANSparkMax m_extendArmMotor1;
  private SparkMaxPIDController m_pidController1;
  private RelativeEncoder m_extendArmEncoder1;
  public DigitalInput input;
  private double m_currentPos_inch = 0.0;
  private double m_desiredPos_inch = 0.0;
  private double m_current_vel_ips = 0.0;
  private static final double GEAR_RATIO = 5.23*5.23;
  public static final double INCHES_PER_ROT = 2.0 * 24.0 * 5.0 / (25.4 * GEAR_RATIO); //stages * pulley teeth * mm per tooth / (mm per inch * gear ratio)
  public boolean m_homed = false;
  private boolean m_enabled = false;
  private Logger logger = Logger.getInstance();


  public ExtendArmSubsystem() {
    // TODO CHANGE SPARKMAX CANID TO USE CONSTANTS
    m_extendArmMotor1 = new CANSparkMax(16, CANSparkMax.MotorType.kBrushless);

    input = new DigitalInput(3);

    homing();

    if (m_extendArmMotor1 != null) {
      m_pidController1 = m_extendArmMotor1.getPIDController();
      m_extendArmEncoder1 = m_extendArmMotor1.getEncoder();
      //m_extendArmEncoder1.setPosition(2.0 / INCHES_PER_ROT);

      m_pidController1.setP(ExtendArmConstants.kp);

      m_pidController1.setOutputRange(ExtendArmConstants.kMinOutput, ExtendArmConstants.kMaxOutput);
      //m_pidController1.setReference(0.0, ControlType.kPosition);
      m_enabled = true;
    }
  }


  @Override
  public void periodic() {
    boolean moving = true;
    if (m_enabled == true) {
      if(m_homed == true)
      {
        if (m_desiredPos_inch > ExtendArmConstants.MAX_EXTEND_INCH) {      //if desired pos for arm is greater than 38 make it 38 and if less than 0 inches make it 0
          m_desiredPos_inch = ExtendArmConstants.MAX_EXTEND_INCH;
        } else if (m_desiredPos_inch < 0) {
          m_desiredPos_inch = 0;
        }
  
        double delta = (m_desiredPos_inch - m_currentPos_inch);
        double v = m_current_vel_ips;
  
        boolean invert = false;
  
        if (delta < 0.0) {
          invert = true;
          delta = -delta;
          v = -v;
        }
  
        double t = Math.sqrt(2.0*delta / ExtendArmConstants.ACCEL_MAX_A_IPS2);
  
        double speed_limit = ExtendArmConstants.ACCEL_MAX_A_IPS2 * (t - ExtendArmConstants.SECONDS_PER_TICK);
  
        if (speed_limit > ExtendArmConstants.ACCEL_MAX_V_IPS) {
          speed_limit = ExtendArmConstants.ACCEL_MAX_V_IPS;
        }
        //adjust speed
        if (v >= speed_limit) {
          //slow down
          v -= ExtendArmConstants.ACCEL_MAX_A_IPS2 * ExtendArmConstants.SECONDS_PER_TICK;
        }
        else if (v + ExtendArmConstants.ACCEL_MAX_A_IPS2 * ExtendArmConstants.SECONDS_PER_TICK < speed_limit) {
          //speed up
          v += ExtendArmConstants.ACCEL_MAX_A_IPS2 * ExtendArmConstants.SECONDS_PER_TICK;
        }
        else {
          //maintain speed
        }
        
        double dx = v * ExtendArmConstants.SECONDS_PER_TICK;
  
        if (invert) {
          m_currentPos_inch += -dx;
          m_current_vel_ips = -v;
        }
        else {
          m_currentPos_inch += dx;
          m_current_vel_ips = v;
        }
  
        double newDeltaX = m_desiredPos_inch - m_currentPos_inch;
        if ((Math.abs(m_current_vel_ips) < ExtendArmConstants.MIN_VEL_IPS) &&
            (Math.abs(newDeltaX) < ExtendArmConstants.MIN_X_INCH)) {
          m_currentPos_inch = m_desiredPos_inch;
          m_current_vel_ips = 0;
          moving = false;
        }
  
  
        double position_rot = m_currentPos_inch / INCHES_PER_ROT;
        m_pidController1.setReference(position_rot, ControlType.kPosition);
  
        logger.recordOutput("ARM ROTATIONS", position_rot);

        SmartDashboard.putNumber("ARM ROTATIONS", position_rot);
        SmartDashboard.putNumber("ARM DESIRED INCH", m_desiredPos_inch);
        SmartDashboard.putNumber("ARM CURRENT INCH", m_currentPos_inch);
        SmartDashboard.putNumber("ARM VELOCITY", m_current_vel_ips);
        SmartDashboard.putBoolean("ARM MOVING?", moving);
      }

    } else {
      return;
    }
  }

  public void setPos_inch(double setPoint) {
    if(m_homed == true) {
      m_desiredPos_inch = setPoint;
    }
    else {
      return;
    }

  }

  public double getPos_inch() {
    return m_desiredPos_inch;
  }

  public void homing() {
    while(input.get() == false) {
      m_extendArmMotor1.set(-0.2);
    }

    // m_extendArmEncoder1.setPosition(0);

    m_homed = true;
    m_extendArmMotor1.stopMotor();
  }
}
