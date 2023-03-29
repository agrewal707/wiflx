% This example demonstrates generation of a custom waveform and download to
% an Agilent RF Signal Generator
%
% Copyright 2009 The MathWorks, Inc.

% Clear the workspace
%clear all

% Define the IQ Signal from components
%NUMSAMPLES=5000;
%x_axis = 1:NUMSAMPLES;
%idata=23000 * sin((2*3.14*x_axis)/NUMSAMPLES);
%qdata=23000 * cos((2*3.14*x_axis)/NUMSAMPLES);
%IQData = idata + j.*qdata;

% Define a filename for the data in the ARB
ArbFileName = 'LIQUID_FRAMEGEN64_QPSK_RRC_K2_M7_WFM';

% Open a VISA connection or a raw TCPIP/GPIB connection to the instrument
% deviceObject = visa('agilent','TCPIP0::A-N5182A-80056.dhcp.mathworks.com::inst0::INSTR');
% deviceObject = gpib('agilent',8,19);
deviceObject = tcpip('192.168.1.14',5025);

% Set up the output buffer size to hold at least the number of bytes we are
% transferring
deviceObject.OutputBufferSize = 100000;
% Set output to Big Endian with TCPIP objects, because we do the interleaving 
% and the byte ordering in code. For VISA or GPIB objecs, use littleEndian.
deviceObject.ByteOrder = 'bigEndian';

% Adjust the timeout to ensure the entire waveform is downloaded before a
% timeout occurs
deviceObject.Timeout = 10.0;

% Open connection to the instrument
fopen(deviceObject);

% Seperate out the real and imaginary data in the IQ Waveform
wave = [real(tx);imag(tx)];
wave = wave(:)';    % transpose the waveform

% Scale the waveform
tmp = max(abs([max(wave) min(wave)]));
if (tmp == 0)
    tmp = 1;
end
% ARB binary range is 2's Compliment -32768 to + 32767
% So scale the waveform to +/- 32767 not 32768
scale = 2^15-1;
scale = scale/tmp;
wave = round(wave * scale);
modval = 2^16;
% Get it from double to unsigned int and let the driver take care of Big
% Endian to Little Endian for you  Look at ESG in Workspace.  It is a
% property of the VISA driver.
wave = uint16(mod(modval + wave, modval));

% Some more commands to make sure we don't damage the instrument
fprintf(deviceObject,':OUTPut:STATe OFF')
fprintf(deviceObject,':SOURce:RADio:ARB:STATe OFF')
fprintf(deviceObject,':OUTPut:MODulation:STATe OFF')

% Set the instrument source freq
fprintf(deviceObject, 'SOURce:FREQuency 757500000');
% Set the source power
fprintf(deviceObject, 'POWer -60');

% Write the data to the instrument
n = size(wave);
sprintf('Starting Download of %d Points\n',n(2)/2)
binblockwrite(deviceObject,wave,'uint16',[':MEM:DATA:UNProtected "WFM1:' ArbFileName '",']);
% Write out the ASCII LF character
fprintf(deviceObject,'');

% Wait for instrument to complete download
% If you see a "Warning: A timeout occurred before the Terminator was reached." 
% warning you will need to adjust the deviceObject.Timeout value until no
% warning results on execution
commandCompleted = query(deviceObject,'*OPC?');

% Some more commands to start playing back the signal on the instrument
fprintf(deviceObject,':SOURce:RADio:ARB:STATe ON')
fprintf(deviceObject,':OUTPut:MODulation:STATe ON')
fprintf(deviceObject,':OUTPut:STATe ON')
fprintf(deviceObject,[':SOURce:RADio:ARB:WAV "ARBI:' ArbFileName '"']);

% Close the connection to the instrument
fclose(deviceObject); delete(deviceObject); clear deviceObject
