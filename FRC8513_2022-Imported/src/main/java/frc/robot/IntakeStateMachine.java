package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.*;

public class IntakeStateMachine {

  public int intakeState;
  double upperSensorDistance;
  double lowerSensorDistance;
  int upperIntakeSensorTHold = 15;
  int lowerIntakeSensorTHold = 15;
  Robot thisRobot;

  public IntakeStateMachine(Robot thisRobotParameter) {
    thisRobot = thisRobotParameter;
    intakeInit();
  }

  public void intakeInit() {
  }

  public void setState(int wantedState) {
    intakeState = wantedState;
  }

  public void updateState()
  {
    /*try { //to avoid getting stuck in case 0 when the sensors are not hooked up or they do not see a ball
      upperSensorDistance=upperIntakeSensor.GetRange();
      lowerSensorDistance=lowerIntakeSensor.GetRange();
    }
    catch(Exception e) {
      upperSensorDistance=-1;
      lowerSensorDistance=-1;
    }*/
    SmartDashboard.putNumber("upperIntakeSensor", upperSensorDistance);
    SmartDashboard.putNumber("lowerIntakeSensor", lowerSensorDistance);
    switch(intakeState){
    case 0:
    break;
    case 1: //do nothing
      thisRobot.lowerMotorPower=0;
      thisRobot.upperMotorPower=0;
      thisRobot.flywheelMotorPower=0;
      if(thisRobot.joystick.getRawButtonPressed(1))
      {
        intakeState=3;
      }
      if(thisRobot.joystick.getRawButtonPressed(5))
      {
        intakeState=5;
      }
      if(thisRobot.joystick.getRawButtonPressed(2))
      {
        intakeState=2;
      }
      if(thisRobot.joystick.getRawButtonPressed(6))
      {
        intakeState=6;
      }
      if(thisRobot.joystick.getRawButtonPressed(3))
      {
        intakeState=7;
      }
    break;
    case 2: //outake just lower
      thisRobot.lowerMotorPower=-1;
      thisRobot.upperMotorPower=0;
      thisRobot.flywheelMotorPower=0;
      if(thisRobot.joystick.getRawButtonReleased(2))
      {
        intakeState=1;
      }
      if(thisRobot.joystick.getRawButtonPressed(4))
      {
        intakeState=1;
      }
    break;
    case 3: //both elevators on
      thisRobot.lowerMotorPower=1;
      thisRobot.upperMotorPower=1;
      thisRobot.flywheelMotorPower=0;
      if(thisRobot.joystick.getRawButtonPressed(4))
      {
        intakeState=1;
      }
      if(upperSensorDistance<upperIntakeSensorTHold)
      {
        intakeState=4;
      }
    break;
    case 4: //lower elevator on
     thisRobot.lowerMotorPower=1;
     thisRobot.upperMotorPower=0;
     thisRobot.flywheelMotorPower=0;
     if(thisRobot.joystick.getRawButtonPressed(4))
      {
        intakeState=1;
      }
      if(thisRobot.joystick.getRawButtonPressed(6))
      {
        intakeState=6;
      }
      if(lowerSensorDistance<lowerIntakeSensorTHold)
      {
        intakeState=1;
      }
    break;
    case 5: //forced intake state
      thisRobot.lowerMotorPower=1;
      thisRobot.upperMotorPower=1;
      thisRobot.flywheelMotorPower=0;
      if(thisRobot.joystick.getRawButtonReleased(5))
      {
        intakeState=1;
      }
    break;
    case 6: //manually shoot
      thisRobot.lowerMotorPower=1;
      thisRobot.upperMotorPower=1;
      thisRobot.flywheelMotorPower=-1;
      if(thisRobot.joystick.getRawButtonReleased(6))
      {
        intakeState=1;
      }
      break;
    case 7: //outtake if the wrong color ball
      thisRobot.lowerMotorPower=-1;
      thisRobot.upperMotorPower=-1;
      thisRobot.flywheelMotorPower=0;
      if(thisRobot.joystick.getRawButtonReleased(3)) //manual
      {
        intakeState=1;
      }
      //add automatic code once color sensor is set up
      break;
    default: 
      intakeState=1;
    break;
  }
  SmartDashboard.putNumber("intake state", intakeState);
}

public void updateMotorPower()
{
  thisRobot.m_lowerIntakeMotor.set(thisRobot.lowerMotorPower);
  thisRobot.m_upperIntakeMotor.set(thisRobot.upperMotorPower);
  thisRobot.m_flywheelMotor.set(thisRobot.flywheelMotorPower);
  SmartDashboard.putNumber("flywheelMotorPower", thisRobot.upperMotorPower); 
  SmartDashboard.putNumber("lowerMotorPower", thisRobot.upperMotorPower); 
  SmartDashboard.putNumber("upperMotorPower", thisRobot.upperMotorPower); 
}
}