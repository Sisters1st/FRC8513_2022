// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the manifest
 * file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  public DifferentialDrive m_myRobot;
  public Joystick joystick;
  public Joystick joystickBlue;
  // initializing motors
  public static final int leftMotorID1 = 4;
  public static final int leftMotorID2 = 5;
  public static final int rightMotorID1 = 2;
  public static final int rightMotorID2 = 3;
  public static final int lowerIntakeID = 9;
  public static final int upperIntakeID = 6;
  public static final int flywheelMotorID = 8;
  public static final int climberMotorID = 7;
  // initializing Can Sparks to
  public CANSparkMax m_leftMotor1;
  public CANSparkMax m_leftMotor2;
  public CANSparkMax m_rightMotor1;
  public CANSparkMax m_rightMotor2;
  public CANSparkMax m_lowerIntakeMotor;
  public CANSparkMax m_upperIntakeMotor;
  public CANSparkMax m_flywheelMotor;
  public CANSparkMax m_climberMotor;
  // motor controller groups
  public MotorControllerGroup m_left;
  public MotorControllerGroup m_right;
  // variables used in the Auto class
  public final Timer m_timer = new Timer();
  public double Auto = 0;
  public double autoStartingAngle;
  public double currentAngle;
  public double leftEncoderPosition = 0;
  public double rightEncoderPosition = 0;
  public double autoGoalAngle = 0;
  public double autoAction = 0;
  public double autoStep = 0;
  public int autoDashboard = 0;
  public double autoAngleTHold = 4;
  public double tHoldCounter;
  public double tHoldCounterTHold = 20;
  public boolean autoActionIsDone = false;
  public double autoGoalDistance;
  public double autoDistanceTHold = .2;
  public double autoGoalSpeed;
  public double autoStartTime = 0;
  public double autoWaitTime = 0;
  // turn PID variables
  double kP_turn = .06;
  double kI_turn = 0.0001;
  double kD_turn = 0.004;
  public PIDController turnPID = new PIDController(kP_turn, kI_turn, kD_turn);
  // straight PID variables
  double kP_straight = 1;
  double kI_straight = 0.1;
  double kD_straight = 0;
  public PIDController straightPID = new PIDController(kP_straight, kI_straight, kD_straight);
  // distance PID variables
  double kP_distance = 1;
  double kI_distance = 0;
  double kD_distance = .001;
  public PIDController distancePID = new PIDController(kP_distance, kI_distance, kD_distance);
  // sensors
  public AHRS ahrs;
  public RelativeEncoder leftEncoder;
  public RelativeEncoder rightEncoder;
  public double currentPosition;
  // intake state machine
  public double lowerMotorPower;
  public double upperMotorPower;
  public double flywheelMotorPower;
  // instantiating the classes
  public frc.robot.Auto autoController = new frc.robot.Auto(this);
  public frc.robot.Teleop teleopController = new frc.robot.Teleop(this);
  public frc.robot.IntakeStateMachine intakeStateController = new frc.robot.IntakeStateMachine(this);

  public boolean getTriggerPressed() {
    return autoActionIsDone;

  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftMotor1 = new CANSparkMax(leftMotorID1, MotorType.kBrushed);
    m_leftMotor2 = new CANSparkMax(leftMotorID2, MotorType.kBrushed);
    m_rightMotor1 = new CANSparkMax(rightMotorID1, MotorType.kBrushed);
    m_rightMotor2 = new CANSparkMax(rightMotorID2, MotorType.kBrushed);
    m_lowerIntakeMotor = new CANSparkMax(lowerIntakeID, MotorType.kBrushed);
    m_upperIntakeMotor = new CANSparkMax(upperIntakeID, MotorType.kBrushed);
    m_flywheelMotor = new CANSparkMax(flywheelMotorID, MotorType.kBrushed);
    m_climberMotor = new CANSparkMax(climberMotorID, MotorType.kBrushed);
    CameraServer.startAutomaticCapture();
    leftEncoder = m_leftMotor1.getEncoder(Type.kQuadrature, 8192);
    rightEncoder = m_rightMotor1.getEncoder(Type.kQuadrature, 8192);
    rightEncoder.setInverted(true);
    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus. */
      /* Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB */
      /*
       * See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for
       * details.
       */
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
    }
    m_left = new MotorControllerGroup(m_leftMotor1, m_leftMotor2);
    m_right = new MotorControllerGroup(m_rightMotor1, m_rightMotor2);
    m_myRobot = new DifferentialDrive(m_left, m_right);
    joystick = new Joystick(0);
    joystickBlue = new Joystick(1);
    m_right.setInverted(true);
  }

  @Override
  public void robotPeriodic() {
    // Smart Dashboard variables
    // pdp, motor power, motor current, angle indicator, clean up dashboard
    currentAngle = ahrs.getAngle();
    leftEncoderPosition = leftEncoder.getPosition();
    rightEncoderPosition = rightEncoder.getPosition();
    //currentPosition = (leftEncoderPosition + rightEncoderPosition) / 2;
    currentPosition = rightEncoderPosition;
    turnPID.setIntegratorRange(-.3, .3); //clamp more
    // putting variables on the Smart Dashboard
    SmartDashboard.putNumber("current angle", currentAngle); // put the value of the current angle on the Smart
                                                             // Dashboard
    SmartDashboard.putNumber("leftEncoder", leftEncoderPosition); // put the value of the left sensor on the Smart
                                                                  // Dashboard
    SmartDashboard.putNumber("rightEncoder", rightEncoderPosition); // put the value of the right sensor on the Smart
                                                                    // Dashboard
    SmartDashboard.putNumber("autoGoalAngle", autoGoalAngle); // put the value of the auto goal angle on the Smart
                                                              // Dashboard
    SmartDashboard.putNumber("autoDashboard", autoDashboard); // put the value of the autoDashboard on the Smart
                                                              // Dashboard
    SmartDashboard.putNumber("autoTHoldCounter", tHoldCounter);
    SmartDashboard.putNumber("Left Motor Power", m_leftMotor1.get());
    SmartDashboard.putNumber("Right Motor Power", m_rightMotor1.get());
 
  }

  /** This function is run once at the beginning of each autonomous mode. */
  @Override
  public void autonomousInit() {
    autoController.autoInit();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    autoController.autoPeriodic();
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    teleopController.teleInit();
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    teleopController.telePeriodic();
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

}