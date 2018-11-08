% highByte = hex2dec('40');
% lowByte = hex2dec('41');

highByte = hex2dec('C3');
lowByte = hex2dec('60');

mAz = (highByte)*(2^8) + lowByte;
Az = double(M20160624_imuConvert(mAz,2))
                        