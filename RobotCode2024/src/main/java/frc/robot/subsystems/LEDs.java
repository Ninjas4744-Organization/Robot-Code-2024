// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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
      new AddressableLED(Constants.LEDs.FL_PORT),
      new AddressableLED(Constants.LEDs.FR_PORT),
      new AddressableLED(Constants.LEDs.BL_PORT),
      new AddressableLED(Constants.LEDs.BR_PORT),
    };
    _ledBuffers = new AddressableLEDBuffer[]{
      new AddressableLEDBuffer(Constants.LEDs.LED_LENGTH),
      new AddressableLEDBuffer(Constants.LEDs.LED_LENGTH),
      new AddressableLEDBuffer(Constants.LEDs.LED_LENGTH),
      new AddressableLEDBuffer(Constants.LEDs.LED_LENGTH),
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
   public List<Color> updatePos(){
    double _delta_x = _autonStart.get().getX();
    double _delta_y = _autonStart.get().getY();
    List<Color> results = new ArrayList<Color>();

    if(_delta_x < 0){
    results.set(Constants.LEDs.FL_LED_INDEX,Color.kRed);
    results.set(Constants.LEDs.BL_LED_INDEX,Color.kRed);
    }
    else if(_delta_x > 0){
          results.set(Constants.LEDs.FR_LED_INDEX,Color.kRed);
          results.set(Constants.LEDs.BR_LED_INDEX,Color.kRed);
          
    }

    if(_delta_y < 0){
      results.set(Constants.LEDs.FL_LED_INDEX,Color.kRed);
    results.set(Constants.LEDs.FR_LED_INDEX,Color.kRed);
    }
    else if(_delta_y > 0){
          results.set(Constants.LEDs.BR_LED_INDEX,Color.kRed);
          results.set(Constants.LEDs.BL_LED_INDEX,Color.kRed);
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
   private void rainbow(AddressableLEDBuffer m_ledBuffer) {
    double m_rainbowFirstPixelHue = 0;
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final int hue = (int)(m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }
   public void autonSetUp(){
    Color rotateLedsData = updateRot();
      List<Color> positionLedsData = updatePos();

    for (int i = 0; i  < _ledBuffers.length; i++) {
      if(rotateLedsData == Color.kGreen && positionLedsData.indexOf(Color.kRed) == -1){
        rainbow(_ledBuffers[i]);
      }
      else{
        setRotLEDs(_ledBuffers[i], rotateLedsData);
      setPoseLEDs(_ledBuffers[i], positionLedsData.get(i));
      }
    
    
    _leds[i].setData(_ledBuffers[i]);
    _leds[i].start();
    }
   }
  @Override
  public void periodic() {
      if (RobotModeTriggers.disabled().getAsBoolean()){
        autonSetUp();
      }

  }
}
