// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;

  private static final int leftDeviceID1 = 2; 
  private static final int leftDeviceID2 = 3; 
  private static final int rightDeviceID1 = 4; 
  private static final int rightDeviceID2 = 5;
  private CANSparkMax m_leftMotor1;
  private CANSparkMax m_leftMotor2;
  private CANSparkMax m_rightMotor1;
  private CANSparkMax m_rightMotor2;

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftMotor1 = new CANSparkMax(leftDeviceID1, MotorType.kBrushed);
    m_leftMotor2 = new CANSparkMax(leftDeviceID2, MotorType.kBrushed);
    m_rightMotor1 = new CANSparkMax(rightDeviceID1, MotorType.kBrushed);
    m_rightMotor2 = new CANSparkMax(rightDeviceID2, MotorType.kBrushed);

    m_leftMotor2.follow(m_leftMotor1);
    m_leftMotor1.setInverted(true);

    m_rightMotor2.follow(m_rightMotor1);
    m_rightMotor2.setInverted(true);

    m_myRobot = new DifferentialDrive(m_leftMotor1, m_rightMotor1);
    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);
  }

  @Override
  public void teleopPeriodic() {
    m_myRobot.tankDrive(m_leftStick.getY(), m_rightStick.getY());
  }
}
