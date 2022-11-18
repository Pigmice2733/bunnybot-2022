// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDLights extends SubsystemBase {
  private AddressableLED lights = new AddressableLED(0);
  private AddressableLEDBuffer buffer = new AddressableLEDBuffer(10);

  //NetworkTableEntry r = Shuffleboard.get

  /** Creates a new LEDlights. */
  public LEDLights() {
    
    lights.setData(buffer);
    lights.start();
  }

  public void periodic() {

  }

  public void setColor(int r, int g, int b) {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, r, g, b);
    }
  }
}
