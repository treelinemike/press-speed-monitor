% IMU Characterization Serial Read Script
% 2016-06-23
% M. Kokko

% Reads serial data from custom IMU characterization control board via
% RS232 serial port.

% close serial port if it was opened previously
if(exist('fid1'))
    fclose(fid1)
end

% restart
close all; clear all; clc;

% options / settings
COM_PORT = 'COM10';
BAUDRATE = 115200;
gyro_range = 1000.0;            % [deg/s] e.g. 1000.0 = +/- 1000deg/s; ST register 0x11 = 0x68
accel_range = 2.0;              % [g] e.g. 2.0 = +/- 2g; ST register 0x10 = 0x60
DLE_byte = hex2dec('10');
STX_byte = hex2dec('02');
ETX_byte = hex2dec('03');
PKT_TYPE_ST_DATA = hex2dec('01');
PKT_TYPE_BOSCH_DATA = hex2dec('02');
PKT_TYPE_ST_CONFIG = hex2dec('A1');
PKT_TYPE_BOSH_CONFIG = hex2dec('A2');
PKT_TYPE_ENCODER_DATA = hex2dec('AA');
mLengthSTData = 23;
mLengthSTConfig = 16;
mLengthEncoder = 11;
serialMaxReadSize = 150;
messageMaxSize = 30;
finalDataMaxSize = 1e6;

% open serial port
fid1 = serial(COM_PORT,'BaudRate',BAUDRATE,'Timeout',0.005);
warning off MATLAB:serial:fread:unsuccessfulRead;
fopen(fid1);
disp('Serial port open, waiting for data!');

% data storage
% TODO: rewrite as circular buffers for speed, portability
allIMUData = NaN(finalDataMaxSize,15);
allIMUDataIdx = uint64(0);
allConfigData = NaN(finalDataMaxSize,6);
allConfigDataIdx = uint64(0);
allEncoderData = NaN(finalDataMaxSize,3);
allEncoderDataIdx = uint64(0);
allMsgNumCols = max([mLengthSTData,mLengthSTConfig,mLengthEncoder]);
allMsg = NaN(finalDataMaxSize,allMsgNumCols);
allMsgIdx = uint64(0);
CRCErrors = 0;

% data storage for initial serial port read
newSerialData = [];
startIdx = 0;

% start reading from serial port
% buffer until DLE, STX pattern is found
while(~startIdx)
    
    % read anything in serial input buffer
    data = fread(fid1,serialMaxReadSize);
    
    % if data present, add to our queue
    if(~isempty(data))
        newSerialData = [newSerialData; data];
        
        % search queue for start of message
        for i = 1:size(newSerialData,1)
            if( (newSerialData(i,1) == DLE_byte) && (newSerialData(i+1,1) == STX_byte) )
                % exclude case where (DLE, DLE, STX) pattern occurs in data
                % otherwise ready to start collecting data from serial port
                if((i == 1) || (newSerialData(i-1,1) ~= DLE_byte))
                    startIdx = i;
                    break;
                end
            end
        end
        
        % if we found the start of a message, discard all previous data
        if(startIdx)
            newSerialData = newSerialData(startIdx:end,:);
        end
    end
    disp(['Waiting buffer size: ' num2str(size(newSerialData,1))]);
end

% newSerialData now contains DLE_byte and STX_byte as its first and second
% elements; enter a continuous loop of processing and reading more data
% from serial port (exit with CTRL-C)
disp('Capturing data from serial port...');
while(1) % TODO: provide more elegant method of exiting loop (multi-threaded GUI?)
    
    msgsInBuffer = 1;
    
    while(msgsInBuffer)
        % find all occurances of DLE, ETX byte sequence in buffer
        DLEETXidx = strfind(newSerialData',[DLE_byte ETX_byte]);
        
        % keep only occurances where ETX is preceeded by an odd number of DLE bytes
        % as this signals end of message (DLE byte before ETX is not stuffed)
        newDLEETXidx = [];
        for i=1:length(DLEETXidx)
            numDLEs = 0;
            thisDLEETXidx = DLEETXidx(i);
            DLECountIdx = thisDLEETXidx;
            
            % count DLE bytes before this ETX byte
            while((DLECountIdx > 0) && (newSerialData(DLECountIdx) == DLE_byte))
                numDLEs = numDLEs + 1;
                DLECountIdx = DLECountIdx -1;
            end
            
            % if odd number of DLE bytes, this DLE, ETX is the end of a message
            if( mod(numDLEs,2) == 1 )
                newDLEETXidx(end+1) = thisDLEETXidx;
            end
        end
        DLEETXidx = newDLEETXidx;
        
        % signal to stop repeating processing loop if there is only one message
        % in buffer
        if(length(DLEETXidx) == 1)
            msgsInBuffer = 0;
        end
        
        % parse a message if we have one
        if(~isempty(DLEETXidx))
            
            % first make sure DLE and STX are first two characters
            DLESTXidx = strfind(newSerialData',[DLE_byte STX_byte]);
            if(isempty(DLESTXidx) || (DLESTXidx(1) ~= 1))
                error('Data Format Error: Starting buffer sequence not DLE, STX');
                    % TODO: recover from this by discarding this message and
                    % waiting for next properly-formed message?
            end
            
            % extract message, skipping stuffed DLE bytes
            thisMsg = NaN(messageMaxSize,1);   % Note: space allocation not really necessary in MATLAB
            thisMsgIdx = 0;
            i = 1;
            while(i < (DLEETXidx(1)+2))
                thisMsgIdx = thisMsgIdx + 1;
                if(thisMsgIdx > messageMaxSize)
                    error('thisMsgIdx overrun!');
                    % TODO: recover from this by discarding this message and
                    % waiting for next properly-formed message?
                end
                thisMsg(thisMsgIdx,1) = newSerialData(i);
                
                % if current and next byte are DLE, skip the next one (this was a stuffed DLE)
                if((newSerialData(i) == DLE_byte) && (newSerialData(i+1) == DLE_byte))
                    i = i+2;
                else
                    i = i+1;
                end
            end
            
            % get size of message
            thisMsgLength = thisMsgIdx;
            
            % remove bytes associated with this message from our queue
            newSerialData = newSerialData((DLEETXidx(1)+2):end);  % TODO: rewrite for speed, portability
            
            % check CRC but don't discard message upon mismatch
            % save data anyway for future reprocessing
            % commenting out because this appears to be very slow
            % causing bytes to be missed from the serial buffer
%             readCRC = thisMsg(thisMsgLength - 2);
%             expectedCRC = M20160624_crcAddBytes(0,thisMsg(3:(thisMsgLength - 3)));
%             if(readCRC ~= expectedCRC)
%                 CRCErrors = CRCErrors + 1;
%                 warning(['CRC mismatch #' num2str(CRCErrors) '! Read: 0x' dec2hex(readCRC) '; Expected: 0x' dec2hex(expectedCRC)]);
%             end
            %disp(['Read CRC: ' num2str(readCRC) '; Expected CRC: ' num2str(expectedCRC)]);
            
            % parse message (note: conversion to engineering units requires
            % knowledge of IMU settings, though raw data are also stored for
            % future reprocessing)
            mType = double(uint8(thisMsg(3)));
            
            % process packet by type-specific methods
            switch(mType)
                
                % process IMU data packets
                case PKT_TYPE_ST_DATA
                    
                    % check length
                    % TODO: include length as a field in protocol and/or check CRC8
                    if( thisMsgLength ~= mLengthSTData)
                        warning(['invalid message length (' num2str(length(thisMsg)) ') for packet type 0x' dec2hex(mType) '!' ])
                    else
                        
                        % extract micro and IMU timestamps
                        mMicroTime = double(uint32(thisMsg(5))*(2^8) + uint32(thisMsg(4)));
                        mIMUTime = double(uint32(thisMsg(8))*(2^16) + uint32(thisMsg(7))*(2^8) + uint32(thisMsg(6)));
                        
                        % extract IMU data
                        stRaw.mGx = thisMsg(10)*(2^8) + thisMsg(9);
                        stRaw.mGy = thisMsg(12)*(2^8) + thisMsg(11);
                        stRaw.mGz = thisMsg(14)*(2^8) + thisMsg(13);
                        stRaw.mAx = thisMsg(16)*(2^8) + thisMsg(15);
                        stRaw.mAy = thisMsg(18)*(2^8) + thisMsg(17);
                        stRaw.mAz = thisMsg(20)*(2^8) + thisMsg(19);
                        stScaled.mGx = double(M20160624_imuConvert(stRaw.mGx,gyro_range));
                        stScaled.mGy = double(M20160624_imuConvert(stRaw.mGy,gyro_range));
                        stScaled.mGz = double(M20160624_imuConvert(stRaw.mGz,gyro_range));
                        stScaled.mAx = double(M20160624_imuConvert(stRaw.mAx,accel_range));
                        stScaled.mAy = double(M20160624_imuConvert(stRaw.mAy,accel_range));
                        stScaled.mAz = double(M20160624_imuConvert(stRaw.mAz,accel_range));
                        
                        % store IMU data
                        allIMUDataIdx = allIMUDataIdx + 1;
                        if(allIMUDataIdx > finalDataMaxSize)
                            error('allIMUDataIdx overrun!');
                        end
                        allIMUData(allIMUDataIdx,:) = [mType mMicroTime mIMUTime stScaled.mGx stScaled.mGy stScaled.mGz stScaled.mAx stScaled.mAy stScaled.mAz stRaw.mGx stRaw.mGy stRaw.mGz stRaw.mAx stRaw.mAy stRaw.mAz];
                        
                        % save raw message bytes for CSV file
                        allMsgIdx = allMsgIdx + 1;
                        allMsg(allMsgIdx,:) = [thisMsg(1:thisMsgLength)' NaN(1,allMsgNumCols-thisMsgLength)];
                        
                    end
                    
                    % process IMU configuration packets
                case PKT_TYPE_ST_CONFIG
                    
                    % check length
                    % TODO: include length as a field in protocol and/or check CRC8
                    if( thisMsgLength ~= mLengthSTConfig)
                        warning(['invalid message length (' num2str(length(thisMsg)) ') for packet type 0x' dec2hex(mType) '!' ])
                    else
                        % extract micro and IMU timestamps
                        mMicroTime = double(uint32(thisMsg(5))*(2^8) + uint32(thisMsg(4)));
                        mIMUTime = double(uint32(thisMsg(8))*(2^16) + uint32(thisMsg(7))*(2^8) + uint32(thisMsg(6)));
                        
                        % extract configuration bytes
                        mST_REG_0h10 = thisMsg(9);
                        mST_REG_0h11 = thisMsg(11);
                        mST_REG_0h5C = thisMsg(13);
                        
                        % store configuration data
                        allConfigDataIdx = allConfigDataIdx + 1;
                        if(allConfigDataIdx > finalDataMaxSize)
                            error('allConfigDataIdx overrun!');
                        end
                        allConfigData(allConfigDataIdx,:) = [mType mMicroTime mIMUTime mST_REG_0h10  mST_REG_0h11 mST_REG_0h5C];
                        
                        % save raw message bytes for CSV file
                        allMsgIdx = allMsgIdx + 1;
                        allMsg(allMsgIdx,:) = [thisMsg(1:thisMsgLength)' NaN(1,allMsgNumCols-thisMsgLength)];
                        
                    end
                    
                    % process encoder data packets
                case PKT_TYPE_ENCODER_DATA
                    
                    % check length
                    % TODO: include length as a field in protocol and/or check CRC8
                    if( thisMsgLength ~= mLengthEncoder)
                        warning(['invalid message length (' num2str(length(thisMsg)) ') for packet type 0x' dec2hex(mType) '!' ])
                    else
                        
                        % extract micro timestamp
                        mMicroTime = double(uint32(thisMsg(5))*(2^8) + uint32(thisMsg(4)));
                        
                        % extract encoder count
                        mEncoderCount = double(uint32(thisMsg(8))*(2^16) + uint32(thisMsg(7))*(2^8) + uint32(thisMsg(6)));
                        
                        % store encoder data
                        allEncoderDataIdx = allEncoderDataIdx + 1;
                        if(allEncoderDataIdx > finalDataMaxSize)
                            error('allEncoderDataIdx overrun!');
                        end
                        allEncoderData(allEncoderDataIdx,:) = [mType mMicroTime mEncoderCount];
                        
                        % save raw message bytes for CSV file
                        allMsgIdx = allMsgIdx + 1;
                        allMsg(allMsgIdx,:) = [thisMsg(1:thisMsgLength)' NaN(1,allMsgNumCols-thisMsgLength)];
                        
                    end
                    
                    % warn about unsupported packet types
                otherwise
                    warning(['Packet type 0x' dec2hex(mtype) ' not supported!']);
                    
            end  % end of switch(mType)
        end % end of if(~isempty(DLEETXidx))
    end % end of while(msgsInBuffer)
    
    % read more data from serial port once all messages in buffer have been
    % processed
    data = [];
    while(isempty(data))
        data = fread(fid1,serialMaxReadSize);
    end
    newSerialData = [newSerialData; data]; % TODO: rewrite for speed, portability
    
end % end of while(1)

%%      *** INTERRUPT PROGRAM HERE WITH CTRL-C ***
%  THEN RUN THE FOLLOWING BLOCK OF CODE WITH CTRL-ENTER

% close serial port
fclose(fid1);

% plotting/analysis options
t_dcoff = 0.5;      % [s] length of time to average (at start of time series) for DC offset calculation
lwidth = 1.6;       % line width for plotting

% trim data to size (remove NaN fields)
allIMUData = allIMUData(1:allIMUDataIdx,:);
allConfigData = allConfigData(1:allConfigDataIdx,:);
allEncoderData = allEncoderData(1:allEncoderDataIdx,:);
allMsg = allMsg(1:allMsgIdx,:);

% save all collected data
[filename,pathname] = uiputfile('.csv');
if((isstr(filename) && isstr(pathname)))
    % create paths for CSV and MAT files
    csvFilename = [pathname filename];
    matFilename = [pathname filename(1:end-3) 'mat'];
    
    % save CSV file
    dlmwrite(csvFilename,allMsg,',');
    
    % save MAT file
    save(matFilename,'allIMUData','allConfigData','allEncoderData','allMsg');
    
    % done saving
    disp('Data saved.');
    selectFile = 0;
else
    % cancel button pressed
    warning('data not saved.');
    selectFile = 0;
end

% post-collection CRC check
% this is very slow, run later in C?
% CRCErrors = 0;
% for i=1:size(allMsg,1)
%     thisMsg = allMsg(i,:)';
%     crcIdx = find(isnan(thisMsg) == 1,1,'first')-3;
%     if(isempty(crcIdx))
%         crcIdx = length(thisMsg)-2;
%     end
%     readCRC = thisMsg(crcIdx);
%     expectedCRC = M20160624_crcAddBytes(0,thisMsg(3:(crcIdx-1)));
% %     disp(['Read CRC: ' num2str(readCRC) '; Expected CRC: ' num2str(expectedCRC)]);
%     if(readCRC ~= expectedCRC)
%         CRCErrors = CRCErrors + 1;
%         warning(['CRC mismatch #' num2str(CRCErrors) ' in message #' num2str(i) '! Read: 0x' dec2hex(readCRC) '; Expected: 0x' dec2hex(expectedCRC)]);
%     end
% end

% extract data for each device and convert to appropriate units
stData = allIMUData(find(allIMUData(:,1) == PKT_TYPE_ST_DATA),:);
disp(sprintf('ST Vector Length: %d',size(stData,1)));
if(size(stData,1) == 0)
    error('Empty dataset!');
end

% extract time vector using timestamps from microcontroller
t_st_micro = unwrap((stData(:,2)*2*pi/65536))*(65536*.000016)/(2*pi);
t0 = t_st_micro(1);
t_st = t_st_micro - t0;

% calculate sampling rates
fs_st = 1/mean(diff(t_st));
fs_st_min = 1/max(diff(t_st));
fs_st_max = 1/min(diff(t_st));
disp(sprintf('ST Sampling Rate [min,mean,max] =    [%06.2f, %06.2f, %06.2f]Hz',fs_st_min,fs_st,fs_st_max));

% extract data
ax_st = stData(:,7);
ay_st = stData(:,8);
az_st = stData(:,9);
Gx_st = stData(:,4);
Gy_st = stData(:,5);
Gz_st = stData(:,6);

% remove dc offset
dcOffPts = floor(fs_st*t_dcoff);
ax_st = ax_st - mean(ax_st(1:dcOffPts));
ay_st = ay_st - mean(ay_st(1:dcOffPts));
az_st = az_st - mean(az_st(1:dcOffPts));
Gx_st = Gx_st - mean(Gx_st(1:dcOffPts));
Gy_st = Gy_st - mean(Gy_st(1:dcOffPts));
Gz_st = Gz_st - mean(Gz_st(1:dcOffPts));

% plot acceleration data
figure;
set(gcf,'Position',[294 167 1167 743]);
ax1(1) = subplot(3,1,1);
hold on; grid on;
plot(t_st,ax_st,'r');
title('\bfAccelerometer Comparison','FontSize',12);
xlabel('\bfTime [s]');
ylabel('\bfX Acceleration [g]');
ax1(2) = subplot(3,1,2);
hold on; grid on;
plot(t_st,ay_st,'r');
xlabel('\bfTime [s]');
ylabel('\bfY Acceleration [g]');
ax1(3) = subplot(3,1,3);
hold on; grid on;
plot(t_st,az_st,'r');
xlabel('\bfTime [s]');
ylabel('\bfZ Acceleration [g]');
linkaxes(ax1,'x');

% plot gyro data
figure;
set(gcf,'Position',[294 167 1167 743]);
ax2(1) = subplot(3,1,1);
hold on; grid on;
plot(t_st,Gx_st,'r');
title('\bfGyro Comparison','FontSize',12);
xlabel('\bfTime [s]');
ylabel('\bfX Angular Velocity [deg/s]');
ax2(2) = subplot(3,1,2);
hold on; grid on;
plot(t_st,Gy_st,'r');
xlabel('\bfTime [s]');
ylabel('\bfY Angular Velocity [deg/s]');
ax2(3) = subplot(3,1,3);
hold on; grid on;
plot(t_st,Gz_st,'r');
xlabel('\bfTime [s]');
ylabel('\bfZ Angular Velocity [deg/s]');
linkaxes(ax2,'x');

% compute and display calibration information if encoder data present
if(~isempty(allEncoderData))
    
    % extract encoder data and compute angular velocity
    t_encoder = unwrap(allEncoderData(:,2)*2*pi/65536)*(65536*.000016)/(2*pi);
    t_encoder = t_encoder - t0;
    theta_encoder = unwrap(allEncoderData(:,3)*(2*pi/(16777216)))*(16777216/(2*pi));
    theta_encoder = theta_encoder*(2*pi/32768);
    w_encoder = [0; diff(theta_encoder)./diff(t_encoder)];
    
    % resample all data to allow plotting encoder vs. gyro and accel
    t0_rs = max(t_encoder(1),t_st(1));
    tf_rs = min(t_encoder(end),t_st(end));
    t_rs = [t0_rs:0.01:tf_rs]';                % TODO: pull-out constant
    st_raw = [Gx_st Gy_st Gz_st ax_st ay_st az_st];
    st_rs = interp1(t_st,st_raw,t_rs,'linear');
    w_rs = interp1(t_encoder,w_encoder,t_rs,'linear');
    Gx_rs = st_rs(:,1);
    Gy_rs = st_rs(:,2);
    Gz_rs = st_rs(:,3);
    ax_rs = st_rs(:,4);
    ay_rs = st_rs(:,5);
    az_rs = st_rs(:,6);
    
    %     % check resampling
    %     figure;
    %     hold on; grid on;
    %     plot(t_st,Gz_st,'r');
    %     plot(t_encoder,w_encoder,'b');
    %     plot(t_rs,Gz_rs,'mx','MarkerSize',8);
    %     plot(t_rs,w_rs,'mx','MarkerSize',8);
    %     xlabel('\bfTime [s]');
    %     ylabel('\bfAngular Velocity [deg/s]');
    
    % plot gyro calibration data
    figure;
    set(gcf,'Position',[54 121 1788 834]);
    subplot(2,3,1);
    hold on; grid on;
    plot(t_rs,Gx_rs,'r');
    plot(t_rs,w_rs*180/pi,'b');
    xlabel('\bfTime [s]');
    ylabel('\bfAngular Velocity [deg/s]');
    legend('ST G_x','Shaft Velocity','Location','Northwest');
    
    subplot(2,3,2);
    hold on; grid on;
    plot(t_rs,Gy_rs,'r');
    plot(t_rs,w_rs*180/pi,'b');
    xlabel('\bfTime [s]');
    ylabel('\bfAngular Velocity [deg/s]');
    legend('ST G_y','Shaft Velocity','Location','Northwest');
    
    subplot(2,3,3);
    hold on; grid on;
    plot(t_rs,Gz_rs,'r');
    plot(t_rs,w_rs*180/pi,'b');
    xlabel('\bfTime [s]');
    ylabel('\bfAngular Velocity [deg/s]');
    legend('ST G_z','Shaft Velocity','Location','Northwest');
    
    subplot(2,3,4);
    hold on; grid on;
    x = w_rs*180/pi;
    y = Gx_rs;
    p = polyfit(x,y,1);
    x2 = [min(x) max(x)];
    y2 = p(1).*x2 + p(2);
    yr = y-(p(1).*x+p(2));
    SSresid = sum(yr.^2);
    SStotal = (length(y)-1)*var(y);
    rsq = 1-SSresid/SStotal;
    plot(x,y,'m.','MarkerSize',10);
    xlabel('\bfShaft Angular Velocity [deg/s]');
    ylabel('\bfGyro X Angular Velocity [deg/s]');
    h1 = plot(x2,y2,'k-','LineWidth',lwidth);
    legend(h1,sprintf('y = %+0.3fx%+0.3f; r^2 = %0.2f',p(1),p(2),rsq));
    
    subplot(2,3,5);
    hold on; grid on;
    x = w_rs*180/pi;
    y = Gy_rs;
    p = polyfit(x,y,1);
    x2 = [min(x) max(x)];
    y2 = p(1).*x2 + p(2);
    yr = y-(p(1).*x+p(2));
    SSresid = sum(yr.^2);
    SStotal = (length(y)-1)*var(y);
    rsq = 1-SSresid/SStotal;
    plot(x,y,'m.','MarkerSize',10);
    xlabel('\bfShaft Angular Velocity [deg/s]');
    ylabel('\bfGyro Y Angular Velocity [deg/s]');
    h1 = plot(x2,y2,'k-','LineWidth',lwidth);
    legend(h1,sprintf('y = %+0.3fx%+0.3f; r^2 = %0.2f',p(1),p(2),rsq));
    
    subplot(2,3,6);
    hold on; grid on;
    x = w_rs*180/pi;
    y = Gz_rs;
    p = polyfit(x,y,1);
    x2 = [min(x) max(x)];
    y2 = p(1).*x2 + p(2);
    yr = y-(p(1).*x+p(2));
    SSresid = sum(yr.^2);
    SStotal = (length(y)-1)*var(y);
    rsq = 1-SSresid/SStotal;
    plot(x,y,'m.','MarkerSize',10);
    xlabel('\bfShaft Angular Velocity [deg/s]');
    ylabel('\bfGyro Z Angular Velocity [deg/s]');
    h1 = plot(x2,y2,'k-','LineWidth',lwidth);   
    legend(h1,sprintf('y = %+0.3fx%a+0.3f; r^2 = %0.2f',p(1),p(2),rsq));
    
    % TODO: add accelerometer calibration/alignment
    
end  % end of calibration

