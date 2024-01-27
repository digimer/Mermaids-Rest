#!/usr/bin/perl
#
# This is for flushing out the math for 'helm-control' at the command line.
# 
#  Mermaid's Rest - Helm Controls
#
#  - Madison Kelly - digital.mermaid@gmail.com
#  - Last Updated  - Jan. 26, 2024

use strict;
use warnings;
use POSIX;

# # Min Forward - 550
# # Max Forward - 765 - 713/4086 ok - 714/4098 overflows, 751/4095 hard set 
# # neutral     - 2010 = 2.500v
# # Min Reverse - 465
# # Max Reverse - 250

### Set static values
# DAC ranges - It should be 000h = 0v, FFFh = 5000mv, but a given DAC can vary
our $dacMinimum      = 0;
our $dacMaximum      = 4095;
our $dacNeutral      = 2010;	# Tested on this DAC, not mathmatically accurate
our $dacNeutralStart = 1965;
our $dacNeutralEnd   = 2130;

# Voltage Range
our $dacVoltageMaximum = 5000;
our $davVoltageMinimum = 0;

# Reverse starts at high pot and counts down
our $reverseMaximum = 265;
our $reverseMinimum = 465;

# Forward starts at low pot and counts up
our $forwardMinimum = 550;
our $forwardMaximum = 750;

# This will let us calculate the expected voltage
our $dacVoltageSteps = ($dacVoltageMaximum - $davVoltageMinimum) / $dacMaximum;

# Reverse starts at high pot and counts down
our $reverseSensorSteps = ($reverseMinimum - $reverseMaximum);
our $reverseDACSteps    = ($dacNeutralStart - $dacMinimum);

# Forward starts at low pot and counts up
our $forwardSensorSteps = ($forwardMaximum - $forwardMinimum);
our $forwardDACSteps    = ($dacMaximum - $dacNeutralEnd);

# Calulate the DAC steps per potentiometer steps
our $reverseDACStepsPerSensorSteps = $reverseDACSteps / $reverseSensorSteps;
our $forwardDACStepsPerSensorSteps = $forwardDACSteps / $forwardSensorSteps;

# This is for showing percentages.
our $reversePercentSteps = 100 / $reverseSensorSteps;
our $forwardPercentSteps = 100 / $forwardSensorSteps;

our $mainSwitchVal = 1;

print "Switch: [".$mainSwitchVal."]\n";
print "- Voltage per step: [".$dacVoltageSteps."] mV\n";
print "- Reverse Sensor Steps: [".$reverseSensorSteps."], DAC steps: [".$reverseDACSteps."], DAC Steps/Pot Step: [".$reverseDACStepsPerSensorSteps."], Pot Minimum: [".$reverseMinimum."], Pot Maximum: [".$reverseMaximum."]\n";
print "- Forward Sensor Steps: [".$forwardSensorSteps."], DAC steps: [".$forwardDACSteps."], DAC Steps/Pot Step: [".$forwardDACStepsPerSensorSteps."], Pot Minimum: [".$forwardMinimum."], Pot Maximum: [".$forwardMaximum."]\n";

#exit 0;

foreach my $sensorValue (200..800)
{
	run($sensorValue);
}


sub run
{
	my ($sensorValue) = @_;
	
	my $dacValue = $dacNeutral;  # # Default to 2.5v
	if ($mainSwitchVal == 0) 
	{
		# Switch is off, set DAC to 2010 / output voltage 2.5v to set the controller to neutral
		$dacValue = $dacNeutral;
	}
	elsif (($sensorValue >= $reverseMaximum) && ($sensorValue <= $reverseMinimum))
	{
		# 465 = Min Reverse = DAC 1965 = 2358 mV
		# 265 = Max Reverse = DAC    0 -    0 mV
		$dacValue = (($sensorValue - $reverseMaximum) * $reverseDACStepsPerSensorSteps) + $dacMinimum;
	}
	elsif (($sensorValue >= $forwardMinimum) && ($sensorValue <= $forwardMaximum))
	{
		# 750 = Max Forward = DAC 4095 = 4914 mV
		# 550 = Min Forward = DAC 2130 = 2556 mV
		$dacValue = (($sensorValue - $forwardMinimum) * $forwardDACStepsPerSensorSteps) + $dacNeutralEnd;
	}
	elsif ($sensorValue > $forwardMaximum)
	{
		# Still Max Forward, DAC 4095
		$dacValue = $dacMaximum;
	}
	elsif ($sensorValue < $reverseMaximum)
	{
		# Still Max Reverse, DAC 0
		$dacValue = 0;
	}
	else
	{
		# In the Neutral deadzone, we want 2500v, DAC = 2084 (2500.8 mV)
		$dacValue = $dacNeutral;
	}
	
	# Round the DAC value
	my $IntegerDACValue = int($dacValue + 0.5);
	my $DACVoltage      = $IntegerDACValue * $dacVoltageSteps;
	my $IntegerVoltage  = int($DACVoltage + 0.5);
	print "Sensor: [".$sensorValue."], DAC: [".$IntegerDACValue."], expected Voltage: [".$IntegerVoltage." mV]";
	
	if ($sensorValue <= $reverseMaximum)
	{
		print " - 100\% Reverse";
	}
	elsif ($sensorValue >= $forwardMaximum)
	{
		print " - 100\% Forward";
	}
	elsif (($sensorValue >= $reverseMaximum) && ($sensorValue <= $reverseMinimum))
	{
		my $percentage = int(($reversePercentSteps * ($reverseMinimum - $sensorValue)) + 0.5);
		print " - ".$percentage."\% Reverse";
	}
	elsif (($sensorValue >= $forwardMinimum) && ($sensorValue <= $forwardMaximum))
	{
		my $percentage = int(($forwardPercentSteps * ($sensorValue - $forwardMinimum)) + 0.5);
		print " - ".$percentage."\% Forward";
	}
	else
	{
		print " - Neutral";
	}
	
	if ($IntegerDACValue > $dacMaximum)
	{
		print " - Too Hish!\n";
	}
	elsif (($IntegerDACValue > $dacNeutralStart) && ($IntegerDACValue < $dacNeutralEnd) && ($IntegerDACValue != $dacNeutral))
	{
		print " - Into-Neutral!\n";
	}
	print "\n";
	
	return(0);
}
