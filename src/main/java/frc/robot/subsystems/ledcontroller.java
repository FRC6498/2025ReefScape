// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of the
// WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Removed unused or unresolved import for InterruptBehavior

public class ledcontroller extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */

    public AddressableLED m_led;

    public AddressableLEDBuffer m_ledBuffer;
    public double phase; // controls the color shift in the rainbow
    public double ledSpeed; // speed the LEDs should cycle at while in rainbow mode
    private long lastUpdateTime = 0; // last time the random number was updated
    // random number for LED color
    public int lastR = 0;
    public int lastG = 0;
    public int lastB = 0;

    public ledcontroller() {
        m_led = new AddressableLED(1); // instantiates the LED object
        m_led.setLength(54); // sets the length of the led strip
        m_ledBuffer = new AddressableLEDBuffer(54); // sets the number of leds that should be controlled by the buffer
        m_led.setData(m_ledBuffer); // writes data from the buffer to the LEDS
        m_led.start();

    } // runs the leds with the data from the buffer constantly

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command LedRun(int r, int g, int b) {
        return runOnce(() -> {
            lastR = r;
            lastG = g;
            lastB = b;
            for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, r, g, b);
            }
            m_led.setData(m_ledBuffer);
        }).withName("LedRun");
    }

    public Command LedStop(int r, int g, int b) {
        return runOnce(() -> {
            for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, 0, 0, 0);

            }
            m_led.setData(m_ledBuffer); // Update the LED strip with the new data
        }).withName("LedRun");
    }

    public Command LedStrobe(int intervalMs) {
        return run(() -> {
            long currentTime = System.currentTimeMillis();
            boolean isOn = (currentTime / intervalMs) % 2 == 0; // Alternate between on and off

            for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                if (isOn) {
                    m_ledBuffer.setRGB(i, lastR, lastG, lastB); // Use the last set color
                } else {
                    m_ledBuffer.setRGB(i, 0, 0, 0); // Turn off the LED
                }
            }

            if (isOn) {
                // Update the last blinking color to ensure it ends on the color
                for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                    lastR = m_ledBuffer.getRed(i);
                    lastG = m_ledBuffer.getGreen(i);
                    lastB = m_ledBuffer.getBlue(i);
                }
            }

            m_led.setData(m_ledBuffer);
        }).repeatedly().withName("LedStrobe");
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a
     * digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    public Command LEDrainbow() {
        return run(() -> {
            long currentTime = System.currentTimeMillis();
            if (currentTime - lastUpdateTime >= 50) { // Check if 50 milliseconds have passed
                lastUpdateTime = currentTime; // Update the last update time

                // Increment the phase to shift the rainbow pattern
                phase += 0.02; // Adjust this value to control the speed of the rainbow
                if (phase >= 1.0) {
                    phase -= 1.0; // Keep phase within [0, 1] range
                }

                // Loop over all LEDs in the buffer and set a rainbow pattern
                for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                    double hue = (i / (double) m_ledBuffer.getLength() + phase) % 1.0; // Calculate hue for each LED
                    int rgb = hsvToRgb(hue, 1.0, 1.0); // Convert HSV to RGB
                    int red = (rgb >> 16) & 0xFF;
                    int green = (rgb >> 8) & 0xFF;
                    int blue = rgb & 0xFF;

                    m_ledBuffer.setRGB(i, red, green, blue); // Set the RGB values for the LED
                }

                m_led.setData(m_ledBuffer); // Update the LED strip with the new data
            }
        }).withName("LEDrainbow");
    }

    /**
     * Converts HSV (Hue, Saturation, Value) to RGB.
     *
     * @param hue        Hue value (0.0 to 1.0)
     * @param saturation Saturation value (0.0 to 1.0)
     * @param value      Value (brightness) (0.0 to 1.0)
     * @return RGB value as an integer (0xRRGGBB)
     */
    private int hsvToRgb(double hue, double saturation, double value) {
        int h = (int) (hue * 6);
        double f = hue * 6 - h;
        double p = value * (1 - saturation);
        double q = value * (1 - f * saturation);
        double t = value * (1 - (1 - f) * saturation);

        double r, g, b;
        switch (h % 6) {
            case 0:
                r = value;
                g = t;
                b = p;
                break;
            case 1:
                r = q;
                g = value;
                b = p;
                break;
            case 2:
                r = p;
                g = value;
                b = t;
                break;
            case 3:
                r = p;
                g = q;
                b = value;
                break;
            case 4:
                r = t;
                g = p;
                b = value;
                break;
            case 5:
            default:
                r = value;
                g = p;
                b = q;
                break;
        }

        int red = (int) (r * 255);
        int green = (int) (g * 255);
        int blue = (int) (b * 255);
        return (red << 16) | (green << 8) | blue;
    }

    public int getLastR() {
        return lastR;
    }

    public int getLastG() {
        return lastG;
    }

    public int getLastB() {
        return lastB;
    }
}
