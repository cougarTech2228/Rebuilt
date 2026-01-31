package frc.robot.subsystems.lights;

import java.util.Random;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
    
    private int counter = 0;

    private final Color DEFAULT_COLOR;
    private final int BUFFER_LENGTH = 147;
    private final AddressableLED mLed;
    private final AddressableLEDBuffer mLedBuffer;

    public Lights(Color defaultColor) {
        this.DEFAULT_COLOR = defaultColor;
        mLed = new AddressableLED(1);
        mLedBuffer = new AddressableLEDBuffer(BUFFER_LENGTH);
    }

    public void setColor(Color color) {
        for (int i = 0; i < mLedBuffer.getLength(); i++) {
            mLedBuffer.setLED(i, color);
        }
        mLed.setData(mLedBuffer);
    }

    public void reset() {
        counter = 0;
        setColor(this.DEFAULT_COLOR);
    }

    public void turnOn() {
        counter = 0;
        mLed.setLength(BUFFER_LENGTH);
        setColor(this.DEFAULT_COLOR);
        mLed.start();
    }

    public void turnOff() {
        mLed.stop();
    }

    public Color getRandomColor() {
        final var rand = new Random();
        final var value = rand.nextInt(0, 8);
        return switch(value) {
            case 0 -> Color.kPink;
            case 1 -> Color.kRed;
            case 2 -> Color.kBlue;
            case 3 -> Color.kOrange;
            case 4 -> Color.kYellow;
            case 5 -> Color.kGreen;
            case 6 -> Color.kPurple;
            case 7 -> Color.kAqua;
            default -> Color.kWhite;
        };
    }

    @Override
    public void periodic() {
        counter++;
        if (counter >= 50) {
            setColor(getRandomColor());
            counter = 0;
            return;
        }
    }
}
