// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constClimber;
import frc.robot.RobotMap.mapClimber;
import frc.robot.RobotPreferences.prefClimber;

public class Climber extends SubsystemBase {

  CANSparkMax climberMotor;

  DigitalInput minSwitch;
  DigitalInput maxSwitch;

  boolean displayOnDashboard;

  public Climber() {
    climberMotor = new CANSparkMax(mapClimber.CLIMBER_MOTOR_CAN, MotorType.kBrushless);

    minSwitch = new DigitalInput(mapClimber.CLIMBER_MINIMUM_SWITCH_DIO);
    maxSwitch = new DigitalInput(mapClimber.CLIMBER_MAXIMUM_SWITCH_DIO);

    configure();

    displayOnDashboard = true;

  }

  public void configure() {

    climberMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    climberMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    climberMotor.setSoftLimit(SoftLimitDirection.kForward, (float) prefClimber.climberAngledMaxPos.getValue());

    climberMotor.setIdleMode(IdleMode.kBrake);
    climberMotor.setInverted(constClimber.INVERTED);
  }

  public void setClimberSpeed(double speed) {
    if ((isMinSwitch() && speed < 0) || (isMaxSwitch() && speed > 0)) {
      speed = 0;
    }
    climberMotor.set(speed);
  }

  public double getClimberEncoderCounts() {
    return climberMotor.getEncoder().getPosition();
  }

  public void resetClimberEncoderCounts() {
    climberMotor.getEncoder().setPosition(0);
  }

  public void neutralMotorOutput() {
    climberMotor.set(0);
  }

  public boolean isMaxSwitch() {
    return !maxSwitch.get();
  }

  public boolean isMinSwitch() {
    return !minSwitch.get();
  }

  public void displayValuesOnDashboard() {
    displayOnDashboard = true;
  }

  public void hideValuesOnDashboard() {
    displayOnDashboard = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (displayOnDashboard) {
      SmartDashboard.putNumber("Climber Encoder Counts",
          getClimberEncoderCounts());
      SmartDashboard.putBoolean("Climber Is At Minimum Switch", isMinSwitch());
      SmartDashboard.putBoolean("Climber Is At Maximum Switch", isMaxSwitch());
    }
    if (isMinSwitch()) {
      resetClimberEncoderCounts();
    }

  }
}