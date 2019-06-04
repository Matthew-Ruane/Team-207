package frc.utility;
import java.util.Arrays;

/**
 *
 * @author Kent Ross
 */


public class RingBuffer {
    final int bufferSize;
    private int[] valueBuffer;
    private long[] timeBuffer;
    private int pos;

    public RingBuffer(int bufferSize, int initialValue) { // buffersize 16, initial value = 0
        this.bufferSize = bufferSize;
        valueBuffer = new int[bufferSize];
        timeBuffer = new long[bufferSize];
        Arrays.fill(valueBuffer, initialValue);
        long nowTime = System.nanoTime();
        Arrays.fill(timeBuffer, nowTime);
    }

    public void Record(int value) {
        pos = Math.floorMod(pos + 1, bufferSize);
        timeBuffer[pos] = System.nanoTime();
        valueBuffer[pos] = value;
    }

    // samplesAgo should be a positive value less than bufferSize.
    public double RateOverSamples(int samplesAgo) {
        int oldPos = Math.floorMod(pos - samplesAgo, bufferSize);
        double timeDelta = timeBuffer[pos] - timeBuffer[oldPos];
        // Time is recorded in nanoseconds, so we multiply by a billion.
        return (valueBuffer[pos] - valueBuffer[oldPos]) / timeDelta * 1e9;
    }
}
