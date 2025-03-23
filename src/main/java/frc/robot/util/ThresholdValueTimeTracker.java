package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;

/*
 * Helper that can be used to track the maxiumum duration that a value equals or exeeds a threshold
 */
public class ThresholdValueTimeTracker {

    class ThresholdValueTimeTrackerResult {
        Time durationExceeded;
        int numberOfReadings;
    }

	class ThresholdExceededTimeDuration {
		Time startTime;
		Time endTime;
        int numberOfReadingsExceeded;

		ThresholdExceededTimeDuration(Time startTime) {
			this.startTime = startTime;
		}

		//If this is false, then we only recorded a single reading exceeding the threshold, so the duration would be zero
		public boolean isClosed() {
			return endTime != null;
		}

		public Time getDuration() {
			if (endTime == null) {
				return Time.ofBaseUnits(0.0, Units.Microseconds);
			}

			return endTime.minus(startTime);
		}
	}

	private double threshold;
	private int numberOfTimesThresholdWasExceeded;
	private int numberOfTimesExceededInCurrentWindow = 0;
	private boolean thresholdWasExceeded;
	private List<ThresholdExceededTimeDuration> timesThresholdWasExceeded;

	public ThresholdValueTimeTracker(double threshold) {
		this.threshold = threshold;
		timesThresholdWasExceeded = new ArrayList<>();
	}

	public ThresholdValueTimeTrackerResult getResult() {
		if (numberOfTimesThresholdWasExceeded == 0) {
			return null;
		}

        ThresholdValueTimeTrackerResult result = new ThresholdValueTimeTrackerResult();
		Time maxDuration = Units.Microseconds.zero();
		for (int i = 0; i < timesThresholdWasExceeded.size(); i++) {
			ThresholdExceededTimeDuration duration = timesThresholdWasExceeded.get(i);
			//ignore single readings
			if (duration.isClosed() && duration.getDuration().compareTo(maxDuration) > 0) {
                result.durationExceeded = duration.getDuration();
                result.numberOfReadings = duration.numberOfReadingsExceeded;
			}
		}

		return result;
	}

	public String describeResult(ThresholdValueTimeTrackerResult result) {
		if (result == null) {
			return "The threshold " + threshold + "was never met";
		}
		if (result.durationExceeded == Units.Microseconds.zero()) {
			return "The threshold " + threshold + " was met for a single reading";
		}

        String durationExceededTime = "null";
        if (result.durationExceeded != null) {
            durationExceededTime = String.valueOf(result.durationExceeded.in(Units.Milliseconds));
        }


		return "The threshold " + threshold + " was met for " + durationExceededTime
            + " ms (" + result.numberOfReadings + " readings)";
	}

    public void reset() {
        numberOfTimesThresholdWasExceeded = 0;
        numberOfTimesExceededInCurrentWindow = 0;
        thresholdWasExceeded = false;
        timesThresholdWasExceeded = new ArrayList<>();
    }

	public void addReading(double value) {
		if (value >= threshold) {
            ThresholdExceededTimeDuration durationItem;
			if (!thresholdWasExceeded) {
				//if threshold was not exceeded on previous reading, start new duration
                durationItem = new ThresholdExceededTimeDuration(RobotController.getMeasureFPGATime());
				timesThresholdWasExceeded.add(durationItem);
				numberOfTimesThresholdWasExceeded++;
			} else {
				//update existing duration
                durationItem = timesThresholdWasExceeded.get(numberOfTimesThresholdWasExceeded - 1);
				durationItem.endTime = RobotController.getMeasureFPGATime();
			}

			thresholdWasExceeded = true;
			numberOfTimesExceededInCurrentWindow++;
            durationItem.numberOfReadingsExceeded = numberOfTimesExceededInCurrentWindow;
		} else {
			if (thresholdWasExceeded) {
				timesThresholdWasExceeded.get(numberOfTimesThresholdWasExceeded - 1).endTime = RobotController.getMeasureFPGATime();
			}

			thresholdWasExceeded = false;
			numberOfTimesExceededInCurrentWindow = 0;
		}
	}
}
