package frc.robot.subsystems.lights;

import java.util.Random;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
    
    private boolean isOn;
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
        final int red = (int) color.red * 255;
        final int green = (int) color.green * 255;
        final int blue = (int) color.blue * 255;
        for (int i = 0; i < mLedBuffer.getLength(); i++) {
            mLedBuffer.setRGB(i, red, green, blue);
        }
        mLed.setData(mLedBuffer);
        System.out.println("Set Lights color to " + color + " length: " +  mLedBuffer.getLength());
    }

    public void reset() {
        counter = 0;
        setColor(this.DEFAULT_COLOR);
    }

    public void turnOn() {
        if (isOn) return;
        counter = 0;
        mLed.setLength(BUFFER_LENGTH);
        mLed.setData(mLedBuffer);
        mLed.start();
        isOn = true;
        System.out.println("Turned on Lights");
    }

    public void turnOff() {
        if (!isOn) return;
        mLed.stop();
        isOn = false;
        System.out.println("Turned off Lights");
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
        if (!isOn) return;
        counter++;
        if (counter >= 50) {
            setColor(getRandomColor());
            counter = 0;
            return;
        }
    }
}
