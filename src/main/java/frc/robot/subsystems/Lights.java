package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private static final int kPort = 9;
    private static final int kLength = 100;
    private Timer m_timer = new Timer();
    private boolean isBlinking = false;
    private int blinkStep = 0;
    private int blinkR, blinkG, blinkB;
    private boolean isIdleOn = false;
    private int idleOffset = 0; // Keeps track of animation step

    public void blinkThrice(int r, int g, int b) {
        blinkR = r;
        blinkG = g;
        blinkB = b;
        isBlinking = true;
        blinkStep = 0;
        m_timer.reset();
        m_timer.start();
    }

    private void updateBlinking() {
        if (!isBlinking) return;
    
        double time = m_timer.get();
    
        if (time >= blinkStep * 0.1 && blinkStep < 6) {
            if (blinkStep % 2 == 0) {
                setSolidColor(blinkR, blinkG, blinkB); // ON
            } else {
                setSolidColor(0, 0, 0); // OFF
            }
            blinkStep++;
        }
    
        if (blinkStep >= 6) {
            isBlinking = false;
            setSolidColor(0, 0, 0); // Ensure LEDs are off after blinking
            m_timer.stop();
        }
    }

    public Lights() {
        m_led = new AddressableLED(kPort);
        m_ledBuffer = new AddressableLEDBuffer(kLength);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
        
        m_timer = new Timer(); // Initialize the timer
        m_timer.start(); // Start the timer
    }
    /* robot centric vs filed oriented = yellow vs blue?%
     * 
     */
    public void toggleIdleStatus() {
        isIdleOn = !isIdleOn;
    }
    
    public void algeaIntake() {
        blinkThrice(51, 163, 145);
    }

    public void coralIntake() {
        blinkThrice(255, 239, 2);
    }
    
    public void reefTracking() {
        blinkThrice(153, 1, 255);
    }

    public void processorTracking() {
        blinkThrice(25,26,137);
    }

    public void scored() {
        blinkThrice(255, 253, 85);
    }

    public void off() {
        setSolidColor(0, 0, 0);
    }

    public void setSolidColor(int r, int g, int b) {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        m_led.setData(m_ledBuffer);
    }

    /** Creates a scrolling blue/yellow/white animation */
    private void updateIdleAnimation() {
        if (!m_timer.isRunning()) {
            m_timer.start(); // Start the timer only once
        }

        if (m_timer.hasElapsed(0.1)) {  // Update every 0.1 seconds
            idleOffset = (idleOffset + 1) % 2; // Cycle through offsets
            for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                int pos = (i + idleOffset) % 2;

                if (pos == 0) {
                    m_ledBuffer.setRGB(i, 0, 2, 61);
                } else if (pos == 1) {
                    m_ledBuffer.setRGB(i, 255, 255, 255);
                }
            }
            m_led.setData(m_ledBuffer);
            m_timer.reset(); // Reset to start counting again
        }
    }

    @Override
    public void periodic() {
        if (isBlinking) {
            updateBlinking();
        } else if (isIdleOn) {
            updateIdleAnimation();
        }

        SmartDashboard.putBoolean("bigD", isIdleOn);
        SmartDashboard.putNumber("timer", m_timer.get());
    }
}