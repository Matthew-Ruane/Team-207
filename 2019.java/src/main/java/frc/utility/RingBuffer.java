package frc.utility;
import java.util.Arrays;

/**
 *
 * @author Kent Ross / Matt Ruane
 */


public class RingBuffer {
    private final int bufferSize;
    private int[] valueBuffer;
    private int pos, sum;

    public RingBuffer(int bufferSize, int initialValue) { // buffersize 16, initial value = 0
        this.bufferSize = bufferSize;
        valueBuffer = new int[bufferSize];
        Arrays.fill(valueBuffer, initialValue);
        pos = 0;
    }
    public void Record(int value) {
        pos = Math.floorMod(pos + 1, bufferSize);
        valueBuffer[pos] = value;
    }
    public double GetBufferAverage() {
        sum = 0;
        for (int i = 0; i < valueBuffer.length; i++) {
            sum = sum + valueBuffer[i];
        }
        double average = sum / valueBuffer.length;
        return average;
    }
}
