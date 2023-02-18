package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShoulderConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// ShoulderSubsystem initializes and sets up two brushless motors and their associated encoders and PID controllers
public class ShoulderSubsystem extends SubsystemBase 
{
  // Declare two instances of the CANSparkMax motor controller class
  private CANSparkMax m_shoulderMotor1 = null;
  public CANSparkMax m_shoulderMotor2 = null;
  // Declare two instances of the SparkMaxPIDController class
  private SparkMaxPIDController m_pidController1 = null;
  private SparkMaxPIDController m_pidController2 = null;
  // Declare two instances of the RelativeEncoder class
  private RelativeEncoder m_shoulderEncoder1 = null;
  private RelativeEncoder m_shoulderEncoder2 = null;

  // Set the current position for the shoulder motors
  public static double m_currentPos = 0.0;
  // Set the desired position that user wants to go to for the shoulder motors
  public static double m_desiredPos = 0.0;

  // Set the number of ticks per second
  private static final double TICKS_PER_SECOND = 50.0; // Revisit this value!!!
  // Rotations it can make (Total_Distance) it can travel
  private static final double TOTAL_DISTANCE = 61.1;
  // Set the time it takes for the shoulder to reach destination
  private static final double SECONDS_TO_MOVE = 1.0; // Revisit this value!!!
  // Calculate the speed of rotation per tick (distance traveled per tick )
  private static final double SPEED_ROT_PER_TICK = ((TOTAL_DISTANCE)) / (SECONDS_TO_MOVE * TICKS_PER_SECOND);

  private boolean m_enabled = false;

  /** Creates a new ShoulderSubsystem. */
  public ShoulderSubsystem() 
  {
    // Initialize the first motor and set its PID controller and encoder

    // creating an instance of CANSparkMax for the shoulder motor with ID ShoulderCanID20
    m_shoulderMotor1 = new CANSparkMax(ShoulderConstants.ShoulderCanID20, CANSparkMax.MotorType.kBrushless);

    // checking if the shoulder motor instance is not null
    if (m_shoulderMotor1 != null) 
    {
      // getting PIDController instance from the shoulder motor
      m_pidController1 = m_shoulderMotor1.getPIDController();
      // getting the encoder instance from the shoulder motor
      m_shoulderEncoder1 = m_shoulderMotor1.getEncoder();
      // setting the encoder position to zero
      //double resetDistance1 = m_shoulderEncoder1.getPosition();
      m_shoulderEncoder1.setPosition(0);

      // setting the P, I, and D values for the PIDController from the ShoulderConstants
      m_pidController1.setP(ShoulderConstants.kp);
      m_pidController1.setI(ShoulderConstants.ki);
      m_pidController1.setD(ShoulderConstants.kd);

      // setting the IZone and FF values for the PIDController from the ShoulderConstants
      m_pidController1.setIZone(ShoulderConstants.kIz);
      m_pidController1.setFF(ShoulderConstants.kFFz);

      // setting the output range for the PIDController from the ShoulderConstants
      m_pidController1.setOutputRange(ShoulderConstants.kMinOutput, ShoulderConstants.kMaxOutput);
      // setting the reference for the PIDController to 0.0, using position control
      /*for (int i = 1; i <= 5; i++) 
      {
        m_pidController1.setReference(resetDistance1 / i, ControlType.kPosition);
        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
        }
      }*/
      m_pidController1.setReference(0.0, ControlType.kPosition);
      // printing a message to indicate the initialization of the shoulder motor 1
      System.out.println("ShoulderMotor1 initialized");
    }

    // Initialize the second motor and set its PID controller and encoder

    // creating an instance of CANSparkMax for the shoulder motor with ID ShoulderCanID21
    m_shoulderMotor2 = new CANSparkMax(ShoulderConstants.ShoulderCanID21, CANSparkMax.MotorType.kBrushless);

    // checking if the shoulder motor instance is not null
    if (m_shoulderMotor2 != null) 
    {
      // getting PIDController instance from the shoulder motor
      m_pidController2 = m_shoulderMotor2.getPIDController();
      // getting the encoder instance from the shoulder motor
      m_shoulderEncoder2 = m_shoulderMotor2.getEncoder();
      // setting the encoder position to zero
      //double resetDistance2 = m_shoulderEncoder1.getPosition();
      m_shoulderEncoder2.setPosition(0);

      // setting the P, I, and D values for the PIDController from the ShoulderConstants
      m_pidController2.setP(ShoulderConstants.kp);
      m_pidController2.setI(ShoulderConstants.ki);
      m_pidController2.setD(ShoulderConstants.kd);

      // setting the IZone and FF values for the PIDController from the ShoulderConstants
      m_pidController2.setIZone(ShoulderConstants.kIz);
      m_pidController2.setFF(ShoulderConstants.kFFz);

      // setting the output range for the PIDController from the ShoulderConstants
      m_pidController2.setOutputRange(ShoulderConstants.kMinOutput, ShoulderConstants.kMaxOutput);
      // setting the reference for the PIDController to 0.0, using position control
      for (int i = 1; i <= 5; i++) 
      /*{
        m_pidController2.setReference(resetDistance2 / i, ControlType.kPosition);
        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
        }
      }*/
      m_pidController2.setReference(0.0, ControlType.kPosition);
      // printing a message to indicate the initialization of the shoulder motor 2
      System.out.println("ShoulderMotor2 initialized");
      m_enabled = true;
    }
  }

  public void setPos(double p_position) 
  {
    m_desiredPos = ((p_position / 360) * 200);
  }

  @Override
  public void periodic() 
  {
    if (m_enabled == true) 
    {
      // This method is called once per scheduler run. It is used to periodically update the motor position to match the desired position.
      if((m_desiredPos > ((110/360)*200)) || (m_desiredPos < ((0/360)*200))) {
        return;
      }
      else {
      // Calculate the difference between the desired position and the current position
      double distance = (m_desiredPos - m_currentPos);

      // Store the difference in a variable named delta
      double delta = distance;

      // Check if delta is greater than the maximum speed(max distance it can travel in a tick) (SPEED_ROT_PER_TICK)
      if (delta > SPEED_ROT_PER_TICK) 
      {
        // If delta is greater than the maximum speed(max distance it can travel in a tick), set delta to the maximum speed(max distance it can travel in a tick)
        delta = SPEED_ROT_PER_TICK;
      }

      // Check if delta is less than the negative of the maximum speed(max distance it can travel in a tick)
      if (delta < -SPEED_ROT_PER_TICK) 
      {
        // If delta is less than the negative of the maximum speed(max rotation it can travel in a tick), set delta to the negative of the maximum speed(max distance it can travel in a tick)
        delta = -SPEED_ROT_PER_TICK;
      }

      // Update the current position by adding delta
      m_currentPos += delta;

      // Set the reference position for the 2 PID controllers in two opposite directions
      m_pidController1.setReference(m_currentPos, ControlType.kPosition);
      m_pidController2.setReference(-m_currentPos, ControlType.kPosition);

      SmartDashboard.putNumber("Shoulder 1 Position", m_shoulderEncoder1.getPosition());
      SmartDashboard.putNumber("Shoulder 2 Position", m_shoulderEncoder2.getPosition());        
      }
    } 
    else 
    {
      return;
    }
  }
}
