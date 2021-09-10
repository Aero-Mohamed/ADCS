function [command, hex, command_] = CommandGenerator(cmd)
    command_ = {'00' '00' '00' '00' '00' '00'};
    command = {'0000', '0000', '0000'};
    for i=1:length(cmd)
        command_(2*i-1) = cmd(i);
        command(i) = cmd(i);
    end
    % Generate CRC
    [crc, hex] = CRC16(hex2dec(command_(1:4)));
    hex_ = strcat(hex(3:4), hex(1:2));
    command{3} = hex_;
end