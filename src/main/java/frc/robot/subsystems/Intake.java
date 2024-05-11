// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.frcteam3255.preferences.SN_DoublePreference;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constIntake;
import frc.robot.RobotMap.mapIntake;

public class Intake extends SubsystemBase {

  TalonSRX roller;
  boolean displayOnDashboard;
  double desiredPercentOutput = 0;

  public Intake() {

    roller = new TalonSRX(mapIntake.INTAKE_MOTOR_CAN);

    displayOnDashboard = true;

    configure();
  }

  public void configure() {
    roller.configFactoryDefault();

    roller.setInverted(constIntake.ROLLER_INVERTED);
  }

  public void setRollerSpeed(SN_DoublePreference speed) {
    desiredPercentOutput = speed.getValue();
    roller.set(ControlMode.PercentOutput, speed.getValue());
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
      SmartDashboard.putNumber("Intake Roller Percent Output", roller.getMotorOutputPercent());
      SmartDashboard.putNumber("Intake Roller Desired Percent Output", desiredPercentOutput);

    }

  }
}
