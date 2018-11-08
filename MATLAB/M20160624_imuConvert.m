function imuVal = M20160624_imuConvert(twosVal, rangeVal)
if(bitshift((bitand(twosVal,hex2dec('8000')) ),-15));
    % negative number
    imuVal = double(bitcmp(uint16(twosVal)-1,'uint16'));
    imuVal = imuVal*(-1*rangeVal/(((2^16)/2)));
else
    % non-negative number
    imuVal = twosVal*(rangeVal/(((2^16)/2)-1));
end
end