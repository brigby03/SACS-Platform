function [theoretical_throttle_percentage] = pwmThrottlePercentage(openTime,closeTime)
theoretical_throttle_percentage = openTime / (openTime + closeTime);
end