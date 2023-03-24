// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GlobalVariables;

public class CANdleSubsystem extends SubsystemBase {

  private final CANdle candle = new CANdle(Constants.CANDLE_ID);

  /** Creates a new CANdleSubsystem. */
  public CANdleSubsystem() {
    candle.configFactoryDefault();
    candle.configStatusLedState(false);
    candle.configLOSBehavior(false);
    candle.configV5Enabled(true);
    candle.configVBatOutput(VBatOutputMode.Off);
    candle.configBrightnessScalar(1);
    candle.configLEDType(LEDStripType.GRB);
    candle.configVBatOutput(VBatOutputMode.On);
    setGamePiece();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  

  public void setLED(int r, int g, int b, int start, int end) {
    int count = end - start + 1;
    candle.setLEDs(r, g, b, 0, start, count);
  }

  public void setAlliance() {
    if(DriverStation.getAlliance() == Alliance.Blue) {
      candle.setLEDs(0, 0, 255, 0, 0, 67);
    }else{
      candle.setLEDs(255, 0, 0, 0, 0, 67);
    }
  }

  public void setGamePiece() {
    if(GlobalVariables.isCone == true) {
      //yellow
      candle.setLEDs(255, 255, 0, 0, 0, 67);
    }else{
      //purple
      candle.setLEDs(148, 0, 211, 0, 0, 67);
    }
  }
}
