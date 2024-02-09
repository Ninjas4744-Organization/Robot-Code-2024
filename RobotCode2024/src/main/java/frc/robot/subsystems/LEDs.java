// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
  AddressableLED[] _leds;
  AddressableLEDBuffer[] _ledBuffers;
  Supplier<Pose2d> _autonStart;
  Supplier<Pose2d> _currentPose;
  /** Creates a new LEDs. */
  public LEDs(Supplier<Pose2d> autonStart,Supplier<Pose2d> currentPose) {
     // PWM port 9
    // Must be a PWM header, not MXP or DIO
    _leds = new AddressableLED[]{
      new AddressableLED(0),
      new AddressableLED(0),
      new AddressableLED(0),
      new AddressableLED(0),
    };
    _ledBuffers = new AddressableLEDBuffer[]{
      new AddressableLEDBuffer(6),
      new AddressableLEDBuffer(6),
      new AddressableLEDBuffer(6),
      new AddressableLEDBuffer(6)
    };

    for (int i = 0; i < _ledBuffers.length; i++) {
      _leds[i].setLength(_ledBuffers[i].getLength());
    }
    
    _autonStart = autonStart;
    _currentPose = currentPose;
    
  }
   public Color updateRot(){
    double delta = _autonStart.get().getRotation().minus(_currentPose.get().getRotation()).getDegrees();
    if(delta < 0){
           return Color.kRed;
    }
    else if(delta > 0){
          return Color.kBlue;
    }
        return Color.kGreen;

   }
   public Color[] updatePos(){
    double _delta_x = _autonStart.get().getX();
    double _delta_y = _autonStart.get().getY();
    Color[] results = new Color[]{
      Color.kGreen,
      Color.kGreen,
      Color.kGreen,
      Color.kGreen,
    };

    if(_delta_x < 0){
    results[Constants.LEDs.FL_LED_INDEX] = Color.kRed;
    results[Constants.LEDs.BL_LED_INDEX] = Color.kRed;
    }
    else if(_delta_x > 0){
          results[Constants.LEDs.FR_LED_INDEX] = Color.kRed;
          results[Constants.LEDs.BR_LED_INDEX] = Color.kRed;
    }

    if(_delta_y < 0){
    results[Constants.LEDs.FL_LED_INDEX] = Color.kRed;
    results[Constants.LEDs.FR_LED_INDEX] = Color.kRed;
    }
    else if(_delta_y > 0){
          results[Constants.LEDs.BR_LED_INDEX] = Color.kRed;
          results[Constants.LEDs.BL_LED_INDEX] = Color.kRed;
    }
    
    return results;
   }
   public void setPoseLEDs(AddressableLEDBuffer _ledBuffer,Color color){
      for (int i = Constants.LEDs.LED_LENGTH/2-1; i < Constants.LEDs.LED_LENGTH; i++) {
        _ledBuffer.setLED(i, color);
      }
      
   }
   public void setRotLEDs(AddressableLEDBuffer _ledBuffer,Color color){
      for (int i = 0; i < Constants.LEDs.LED_LENGTH/2; i++) {
        _ledBuffer.setLED(i, color);
      }
      
   }
  @Override
  public void periodic() {
      Color rotateLedsData = updateRot();
      Color[] positionLedsData = updatePos();

    for (int i = 0; i  < _ledBuffers.length; i++) {
      setRotLEDs(_ledBuffers[i], rotateLedsData);
      setPoseLEDs(_ledBuffers[i], positionLedsData[i]);
    

    _leds[i].setData(_ledBuffers[i]);
    _leds[i].start();
    }
  }
}
