// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDlights extends SubsystemBase {
  private AddressableLED lights;
  private AddressableLEDBuffer lightss;
  /** Creates a new LEDlights. */
  public LEDlights() 
  {
    lights = new AddressableLED(0);
    lightss = new AddressableLEDBuffer(0);
    lights.setData(lightss);
    lights.start();
  }
  private void setColor(int hue){
    for(int i = 0;i < lightss.getLength();i++){
      lightss.setHSV(i, hue, 255, 128);
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
