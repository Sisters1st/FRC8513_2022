// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AnalogInput;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick joystick;

  private static final int leftMotorID1 = 2; 
  private static final int leftMotorID2 = 3; 
  private static final int rightMotorID1 = 4; 
  private static final int rightMotorID2 = 5;
  private static final int mechFinalID1 = 6;
  private static final int mechFinalID2 = 7;
  private CANSparkMax m_leftMotor1;
  private CANSparkMax m_leftMotor2;
  private CANSparkMax m_rightMotor1;
  private CANSparkMax m_rightMotor2;
  private CANSparkMax m_mechID1;
  private CANSparkMax m_mechID2;
  private final Timer m_timer = new Timer();
  private double Auto = 0;
  private final AnalogInput ultrasonic = new AnalogInput(0);
  AHRS ahrs;
  private double autoStartingAngle;
  private double currentAngle;

  /**
   * This function is run when the robot is first started up and should be used for any
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
    m_mechID1 = new CANSparkMax(mechFinalID1, MotorType.kBrushed);
    m_mechID2 = new CANSparkMax(mechFinalID2, MotorType.kBrushed);
    CameraServer.startAutomaticCapture();
    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
      ahrs = new AHRS(SPI.Port.kMXP); 
  } catch (RuntimeException ex ) {}

    m_leftMotor2.follow(m_leftMotor1);

    m_rightMotor2.follow(m_rightMotor1);
    m_rightMotor2.setInverted(true);

    m_myRobot = new DifferentialDrive(m_leftMotor1, m_rightMotor1);
    joystick = new Joystick(0);
  }

  @Override
  public void robotPeriodic(){
    SmartDashboard.putNumber("test", 1);
    Auto = Preferences.getDouble("Auto", 1.0);
    SmartDashboard.putNumber("autoRead", Auto);
    double rawValue = ultrasonic.getValue();
    SmartDashboard.putNumber("ultrasonic", rawValue);
    currentAngle = ahrs.getAngle();
    SmartDashboard.putNumber("angle", currentAngle);
  }
  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
    autoStartingAngle = currentAngle;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    switch((int)Auto) {
      case 0:
        // do nothing auto
        break;
      case 1:
        if (m_timer.get() < 2.0) {
          //m_myRobot.tankDrive(1, 1); // drive forwards half speed

        } else {
          m_myRobot.stopMotor(); // stop robot
        }
        break;
      case 2:
        if (m_timer.get() < 2.0) {
          m_myRobot.tankDrive(-1, -1); // drive backawrds half speed
        } else {
          m_myRobot.stopMotor(); // stop robot
        }
        break;

      case 3:
        if (autoStartingAngle + 90 > currentAngle){
          m_myRobot.tankDrive(-1, 1); 
        }
        else {
          m_myRobot.stopMotor(); // stop robot

        }
        break;
      case 4:
       if (autoStartingAngle - 90 < currentAngle){
        m_myRobot.tankDrive(1, -1); 
      
        }
       else {
        m_myRobot.stopMotor(); // stop robot

       }
       break; 
      case 8:
        if (currentAngle < 360 + autoStartingAngle){
          m_myRobot.tankDrive(1,-1); // spins robot 
        } else {
          m_myRobot.stopMotor(); // stop robot
        }
      default:
        // do nothing
    }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    m_myRobot.tankDrive(joystick.getY(), joystick.getRawAxis(3));
    if(joystick.getRawButtonPressed(6))
    {
      m_mechID1.set(.5);
    }
    if(joystick.getRawButtonPressed(8))
    {
      m_mechID1.set(-.5);
    }
    if(joystick.getRawButtonPressed(7))
    {
      m_mechID2.set(.5);
    }
    if(joystick.getRawButtonPressed(5))
    {
      m_mechID2.set(-.5);
    }
    if(!joystick.getRawButton(6)&&!joystick.getRawButton(8))
    {
      m_mechID1.set(0);
    }
    if(!joystick.getRawButton(5)&&!joystick.getRawButton(7))
    {
      m_mechID2.set(0);
    }
  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
