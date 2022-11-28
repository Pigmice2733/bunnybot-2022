// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDlights extends SubsystemBase {
  private AddressableLED lights = new AddressableLED(0);
  private AddressableLEDBuffer buffer = new AddressableLEDBuffer(10);

  //NetworkTableEntry r = Shuffleboard.get

  /** Creates a new LEDlights. */
  public LEDlights() {
    
    lights.setData(buffer);
    lights.start();
  }

  public void periodic() {

  }

  /** Sets the LED colors with the given RGB values. */
  public void setRGB(int r, int g, int b) {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, r, g, b);
    }
  }

  /** Sets the LED colors to the given hue with the setHSV method.
   * @param hue the hue value to set, with modulo 180
   */
  public void setHue(int hue) {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setHSV(i, (hue % 180), 255, 255);
    }
  }

  /** Sets the LEDs to a rainbow pattern with the given number of rainbow cycles. */
  public void setRainbow(int cycles) {
    this.setRainbow(cycles, 0);
  }

  /** Sets the LEDs to a rainbow pattern with the given number of rainbow cycles.
   * @param cycles the number of rainbow cycles to set
   * @param offset the index in the chain to start the string at
   */
  public void setRainbow(int cycles, int offset) {
    int cycleLength = buffer.getLength() / cycles; // integer divide
    int remainder = buffer.getLength() % cycles;

    for (int i = 0; i < cycles; i++) {
      for (int j = 0; j < cycleLength; j++) {
        buffer.setHSV((j + i*cycleLength), (int) ((j+offset) / cycleLength * 180), 255, 255);
      }
    }
    for (int k = 0; k < remainder; k++) {
      buffer.setHSV((k + cycles*cycleLength), (int) ((k+offset) / cycleLength * 180), 255, 255);
    }
  }
}
