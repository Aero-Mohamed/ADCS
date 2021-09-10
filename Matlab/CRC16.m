function [crc, hex] = CRC16(data)
    crc = 0;
    for i = 1:length(data)
        crc = bitxor( crc, bitshift(data(i),8) );
        for bit = 1:8
            if bitand( crc, bitshift(1, 15) )     % if MSB=1
              crc = bitxor( bitshift(crc,1), hex2dec('1021') );
            else
              crc = bitshift(crc,1);
            end
            crc = bitand( crc, hex2dec('ffff') );  % trim to 16 bits
        end
    end
    hex = dec2hex(crc);
end
