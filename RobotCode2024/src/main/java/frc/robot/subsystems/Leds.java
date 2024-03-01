package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
  private AddressableLED _led;
  private AddressableLEDBuffer _ledBuffer;

  public Leds() {
    _led = new AddressableLED(9);
    _ledBuffer = new AddressableLEDBuffer(36);
    _led.setLength(_ledBuffer.getLength());
    _led.setData(_ledBuffer);
    _led.start();

    for (int i = 0; i < _ledBuffer.getLength(); i++)
      _ledBuffer.setRGB(i, 0, 255, 0);
    _led.setData(_ledBuffer);
  }

  public Command setColor(int r, int g, int b){
    return Commands.run(() -> {
      for (int i = 0; i < _ledBuffer.getLength(); i++)
        _ledBuffer.setRGB(i, r, g, b);
      _led.setData(_ledBuffer);
    }, this);
  }

  public Command setColorBeep(int r, int g, int b, double t){
    return Commands.repeatingSequence(
      Commands.run(() -> {
        for (int i = 0; i < _ledBuffer.getLength(); i++)
          _ledBuffer.setRGB(i, r, g, b);   
        _led.setData(_ledBuffer);
      }, this).withTimeout(t),

      Commands.run(() -> {
        for (int i = 0; i < _ledBuffer.getLength(); i++)
        _ledBuffer.setRGB(i, 0, 0, 0);   
        _led.setData(_ledBuffer);
      }, this).withTimeout(t)
    ).repeatedly();
  }
}