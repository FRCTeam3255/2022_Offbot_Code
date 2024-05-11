// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.frcteam3255.preferences.SN_DoublePreference;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.constTransfer;
import frc.robot.RobotMap.mapTransfer;
import frc.robot.RobotPreferences.prefTransfer;

public class Transfer extends SubsystemBase {

  CANSparkMax entranceWheel;
  TalonSRX bottomBelt;
  TalonSRX topBelt;

  DigitalInput bottomSwitch;
  DigitalInput topSwitch;

  boolean displayOnDashboard;

  public Transfer() {

    entranceWheel = new CANSparkMax(mapTransfer.ENTRANCE_MOTOR_CAN, MotorType.kBrushless);
    bottomBelt = new TalonSRX(mapTransfer.BOTTOM_MOTOR_CAN);
    topBelt = new TalonSRX(mapTransfer.TOP_MOTOR_CAN);

    bottomSwitch = new DigitalInput(mapTransfer.BOTTOM_SWITCH_DIO);
    topSwitch = new DigitalInput(mapTransfer.TOP_SWITCH_DIO);

    displayOnDashboard = true;

    configure();
  }

  public void configure() {
    entranceWheel.restoreFactoryDefaults();
    bottomBelt.configFactoryDefault();
    topBelt.configFactoryDefault();

    entranceWheel.setIdleMode(IdleMode.kBrake);
    bottomBelt.setNeutralMode(NeutralMode.Brake);
    topBelt.setNeutralMode(NeutralMode.Brake);

    entranceWheel.setInverted(constTransfer.ENTRANCE_WHEEL_INVERTED);
  }

  public void setTopBeltSpeed(SN_DoublePreference speed) {
    topBelt.set(ControlMode.PercentOutput, speed.getValue());
  }

  public void setBottomBeltSpeed(SN_DoublePreference speed) {
    bottomBelt.set(ControlMode.PercentOutput, speed.getValue());
  }

  public void setEntranceWheelSpeed(SN_DoublePreference speed) {
    entranceWheel.set(speed.getValue());
  }

  public boolean isTopBallCollected() {
    return (constTransfer.TOP_LIMIT_SWITCH_INVERT) ? !topSwitch.get() : topSwitch.get();
  }

  public boolean isBottomBallCollected() {
    return (constTransfer.BOTTOM_LIMIT_SWITCH_INVERT) ? !bottomSwitch.get() : bottomSwitch.get();
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

      SmartDashboard.putNumber("Transfer Top Belt Speed", topBelt.getMotorOutputPercent());
      SmartDashboard.putNumber("Transfer Bottom Belt Speed", bottomBelt.getMotorOutputPercent());
      SmartDashboard.putNumber("Transfer Entrance Wheel Speed", entranceWheel.getAppliedOutput());

      SmartDashboard.putBoolean("Transfer Is Top Ball Collected", isTopBallCollected());
      SmartDashboard.putBoolean("Transfer Is Bottom Ball Collected", isBottomBallCollected());

      SmartDashboard.putBoolean("Transfer Top Switch RAW", topSwitch.get());
      SmartDashboard.putBoolean("Transfer Bottom Switch RAW", bottomSwitch.get());
    }

  }
}
