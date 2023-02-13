package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShoulderConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// ShoulderSubsystem initializes and sets up two brushless motors and their associated encoders and PID controllers
public class ShoulderSubsystem extends SubsystemBase 
{
  // Declare two instances of the CANSparkMax motor controller class
  private CANSparkMax shoulderMotor1 = null;
  public CANSparkMax shoulderMotor2 = null;
  // Declare two instances of the SparkMaxPIDController class
  private SparkMaxPIDController pidController1 = null;
  private SparkMaxPIDController pidController2 = null;
  // Declare two instances of the RelativeEncoder class
  private RelativeEncoder shoulderRelativeEncoder1 = null;
  private RelativeEncoder shoulderRelativeEncoder2 = null;

  // Set the current position for the shoulder motors
  public static double currentPos = 0.0;
  // Set the desired position that user wants to go to for the shoulder motors
  public static double desiredPos = 0.0;

  // Set the number of ticks per second
  private static final double TICKS_PER_SECOND = 50.0; // Revisit this value!!!
  // Rotations it can make (Total_Distance) it can travel
  private static final double TOTAL_DISTANCE = 61.1;
  // Set the time it takes for the shoulder to reach destination
  private static double SECONDS_TO_MOVE = 1.0; // Revisit this value!!!
  // Calculate the speed of rotation per tick (distance traveled per tick )
  private static final double SPEED_ROT_PER_TICK = ((TOTAL_DISTANCE)) / (SECONDS_TO_MOVE * TICKS_PER_SECOND);

  private boolean m_enabled = false;

  /** Creates a new ShoulderSubsystem. */
  public ShoulderSubsystem() 
  {
    // Initialize the first motor and set its PID controller and encoder

    // creating an instance of CANSparkMax for the shoulder motor with ID ShoulderCanID20
    shoulderMotor1 = new CANSparkMax(ShoulderConstants.ShoulderCanID20, CANSparkMax.MotorType.kBrushless);

    // checking if the shoulder motor instance is not null
    if (shoulderMotor1 != null) 
    {
      // getting PIDController instance from the shoulder motor
      pidController1 = shoulderMotor1.getPIDController();
      // getting the encoder instance from the shoulder motor
      shoulderRelativeEncoder1 = shoulderMotor1.getEncoder();
      // setting the encoder position to zero
      shoulderRelativeEncoder1.setPosition(0);

      // setting the P, I, and D values for the PIDController from the ShoulderConstants
      pidController1.setP(ShoulderConstants.kp);
      pidController1.setI(ShoulderConstants.ki);
      pidController1.setD(ShoulderConstants.kd);

      // setting the IZone and FF values for the PIDController from the ShoulderConstants
      pidController1.setIZone(ShoulderConstants.kIz);
      pidController1.setFF(ShoulderConstants.kFFz);

      // setting the output range for the PIDController from the ShoulderConstants
      pidController1.setOutputRange(ShoulderConstants.kMinOutput, ShoulderConstants.kMaxOutput);
      // setting the reference for the PIDController to 0.0, using position control
      pidController1.setReference(0.0, ControlType.kPosition);
      // printing a message to indicate the initialization of the shoulder motor 1
      System.out.println("ShoulderMotor1 initialized");
    }

    // Initialize the second motor and set its PID controller and encoder

    // creating an instance of CANSparkMax for the shoulder motor with ID ShoulderCanID21
    shoulderMotor2 = new CANSparkMax(ShoulderConstants.ShoulderCanID21, CANSparkMax.MotorType.kBrushless);

    // checking if the shoulder motor instance is not null
    if (shoulderMotor2 != null) 
    {
      // getting PIDController instance from the shoulder motor
      pidController2 = shoulderMotor2.getPIDController();
      // getting the encoder instance from the shoulder motor
      shoulderRelativeEncoder2 = shoulderMotor2.getEncoder();
      // setting the encoder position to zero
      shoulderRelativeEncoder2.setPosition(0);

      // setting the P, I, and D values for the PIDController from the ShoulderConstants
      pidController2.setP(ShoulderConstants.kp);
      pidController2.setI(ShoulderConstants.ki);
      pidController2.setD(ShoulderConstants.kd);

      // setting the IZone and FF values for the PIDController from the ShoulderConstants
      pidController2.setIZone(ShoulderConstants.kIz);
      pidController2.setFF(ShoulderConstants.kFFz);

      // setting the output range for the PIDController from the ShoulderConstants
      pidController2.setOutputRange(ShoulderConstants.kMinOutput, ShoulderConstants.kMaxOutput);
      // setting the reference for the PIDController to 0.0, using position control
      pidController2.setReference(0.0, ControlType.kPosition);
      // printing a message to indicate the initialization of the shoulder motor 2
      System.out.println("ShoulderMotor2 initialized");
      m_enabled = true;
    }
  }

  public void setPos(double position) 
  {
    desiredPos = ((position / 360) * 200);
  }

  @Override
  public void periodic() 
  {
    if (m_enabled == true) 
    {
      // This method is called once per scheduler run. It is used to periodically update the motor position to match the desired position.

      // Calculate the difference between the desired position and the current position
      double distance = (desiredPos - currentPos);

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
      currentPos += delta;

      // Set the reference position for the 2 PID controllers in two opposite directions
      pidController1.setReference(currentPos, ControlType.kPosition);
      pidController2.setReference(-currentPos, ControlType.kPosition);

      SmartDashboard.putNumber("Shoulder 1 Position", shoulderRelativeEncoder1.getPosition());
      SmartDashboard.putNumber("Shoulder 2 Position", shoulderRelativeEncoder2.getPosition());
    } 
    else 
    {
      return;
    }
  }
}
