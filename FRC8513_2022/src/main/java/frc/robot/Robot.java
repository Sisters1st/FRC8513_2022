// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

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
  // variables used in the Auto class
  public final Timer m_timer = new Timer();
  // turn PID variables

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
    
    m_rightMotor1.follow(m_rightMotor2);
    m_leftMotor1.follow(m_leftMotor2);
    
    
    m_myRobot = new DifferentialDrive(m_leftMotor2, m_rightMotor2);
    joystick = new Joystick(0);
  }

  @Override
  public void robotPeriodic() {
 
  }

  /** This function is run once at the beginning of each autonomous mode. */
  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    m_myRobot.arcadeDrive(0.6*joystick.getRawAxis(1),0.6* joystick.getRawAxis(2));
    boolean stop = true;
    if(joystick.getRawButton(5)){
      m_lowerIntakeMotor.set(0.6);
      m_upperIntakeMotor.set(0.6);
      m_flywheelMotor.set(0);
      stop = false;
    }

    if(joystick.getRawButton(6)){
      m_lowerIntakeMotor.set(0.6);
      m_upperIntakeMotor.set(0.8);
      m_flywheelMotor.set(-1);
      stop = false;
    } 
    if(stop){
              m_lowerIntakeMotor.set(0);
      m_upperIntakeMotor.set(0);
      m_flywheelMotor.set(0);
    }
  
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