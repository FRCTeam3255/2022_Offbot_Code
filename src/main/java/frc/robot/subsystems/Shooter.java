// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.frcteam3255.preferences.SN_DoublePreference;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constShooter;
import frc.robot.RobotMap.mapShooter;
import frc.robot.RobotPreferences.prefShooter;

public class Shooter extends SubsystemBase {

  CANSparkMax leadMotor;
  CANSparkMax followMotor;

  double goalSpeed;

  boolean displayOnDashboard;

  public Shooter() {

    leadMotor = new CANSparkMax(mapShooter.LEAD_MOTOR_CAN, MotorType.kBrushless);
    followMotor = new CANSparkMax(mapShooter.FOLLOW_MOTOR_CAN, MotorType.kBrushless);

    configure();

    displayOnDashboard = true;

    goalSpeed = 0;
  }

  public void configure() {

    leadMotor.restoreFactoryDefaults();
    followMotor.restoreFactoryDefaults();

    leadMotor.setInverted(constShooter.LEAD_INVERTED);
    followMotor.setInverted(constShooter.FOLLOW_INVERTED);

    leadMotor.setIdleMode(IdleMode.kCoast);

    followMotor.follow(leadMotor);
  }

  public void setMotorSpeed(double speed) {
    leadMotor.set(speed);
  }

  public void neutralOutput() {
    leadMotor.set(0);
  }

  public double getMotorSpeed() {
    return leadMotor.getAppliedOutput();
  }

  public boolean isMotorAtSpeed() {
    return getMotorErrorToGoalSpeed() < prefShooter.shooterAllowableClosedloopErrorSpeed.getValue();
  }

  private double getMotorErrorToGoalSpeed() {
    return Math.abs(getGoalSpeed() - getMotorSpeed());
  }

  public void setGoalSpeed(double goalSpeed) {
    this.goalSpeed = goalSpeed;
  }

  public void setGoalSpeed(SN_DoublePreference goalSpeed) {
    setGoalSpeed(goalSpeed.getValue());
  }

  public double getGoalSpeed() {
    return goalSpeed;
  }

  public void setMotorSpeedToGoalSpeed() {
    setMotorSpeed(getGoalSpeed());
  }

  public void displayValuesOnDashboard() {
    displayOnDashboard = true;
  }

  public void hideValuesOnDashboard() {
    displayOnDashboard = false;
  }

  @Override
  public void periodic() {

    if (displayOnDashboard) {

      SmartDashboard.putNumber("Shooter Motor Speed", getMotorSpeed());
      SmartDashboard.putNumber("Shooter Motor Percent Output", leadMotor.getAppliedOutput());
      SmartDashboard.putNumber("Shooter Goal Speed", getGoalSpeed());
      SmartDashboard.putBoolean("Shooter Is At Speed", isMotorAtSpeed());
      SmartDashboard.putNumber("Shooter Error to Goal Speed", getMotorErrorToGoalSpeed());

    }

  }
}
