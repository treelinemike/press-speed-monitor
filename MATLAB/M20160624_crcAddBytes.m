function CRC = M20160624_crcAddBytes(CRC, bytesToAdd)

crcPoly = bin2dec('00110001');

if(min(size(bytesToAdd)) ~= 1)
    error('Bad input to crcAddBytes()');
end

for byteNum=1:1:length(bytesToAdd)
    thisByte = bytesToAdd(byteNum);
    
    % iterate through each bit in this byte, from MOST SIGNIFICANT to LEAST SIGNIFICANT
    for bitNum = 7:-1:0
        
        % get the input bit
        thisBit = ~(0==bitand(thisByte,bitshift(1,bitNum,'uint8')));
        
        % determine whether we need to do inversion
        inv_status = xor(thisBit,~(0==bitand(CRC,hex2dec('80'))));
        
        % cycle CRC
        CRC = bitshift(CRC,1,'uint8');
        
        % perform inversion on appropriate bits if necessary
        if(inv_status)
            CRC = bitxor(CRC,crcPoly);
        end
    end
end


end