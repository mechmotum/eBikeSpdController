function bits = bits2volts(bits)
% This function converts bits measured from the Arduino analogRead function
% to volts

bits = 2.*bits.*(5.0/1023.0); % here bits (the output) is actually volts 
  
% Uncomment the following to convert volts to m/s as shown here in the calibration section https://moorepants.github.io/dissertation/davisbicycle.html  
% sf = 2*3.1415/60;     %/* from dissertation */ \
% m = 456.3862;         %/* from dissertation */ \
% b = -1.2846;          %/* from dissertation */ \
% rR = 0.341;           %/* from modeling the human controlled bicycle [m] */ \
% rD = 0.028985;        %/* from dissertation [m] */ \
% rC = 0.333375;
%volts = (sf*(m.*bits+b)*rR*rD)/rC;
end

