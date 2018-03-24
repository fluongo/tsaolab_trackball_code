classdef MotionSensor_FL < handle
    %MotionSensor Handles reading of motion sensor data
    % Create a new MotionSensor object with the COM-port as input argument.
    % The sensor will then be initialized automatically. To use the sensor,
    % first call StartReading(). After this, succesive calls to
    % ReadSensor() will return the accumulated movement data since the last
    % call to ReadSensor().
    
    
    properties
        serialPort = [];
        writeBuffer = uint8(zeros(1,5));
        readBuffer = uint8(zeros(1,5));
        
        sensorInit = 0;
        sensorReading = 0;
        
        initByte = 1;
        getParamByte = 3;
        setParamByte = 4;
        startReadingByte = 5;
        stopReadingByte  = 6;
        getDataByte = 7;
        initResponse = 1;
        
        xScaling = 0;
        yScaling = 0;
    end
    
    methods
        % Object constructor and destructor
        function obj = MotionSensor(portName)
            % MotionSensor Constructor
            % Tries to connect to sensor on port: portName and initializes
            % the sensor.          
            try
                % Open serial port
                obj.serialPort = serial(portName, 'BaudRate',9600);
                fopen(obj.serialPort);
                pause(20);
                
                % Try to send initialization byte
                fwrite(obj.serialPort,obj.initByte,'int8');

                response = fread(obj.serialPort,1,'int8')
                obj.SetParams([40 31]);
                if ( response == obj.initResponse)
                    obj.sensorInit = 1;
                end
                display('Connected to sensor.');
            catch exc
                display('No connection with motion sensor');
                display('Subsequent calls to readsensor will return zero values');
                display(['Error message: ' exc.message]);
            end
        end 
        function delete(obj)
            % MotionSensor Destructor
            % Make sure we terminate connection with the sensor in a
            % correct way
            if (obj.sensorReading == 1)
                obj.StopReading();
            end
            fclose(obj.serialPort);
            display('Disconnected from motion sensor');
        end
        
        % Reading and setting sensor parameters
        function [CPI] = GetCPI(obj)
            % MotionReader GetCPI
            % returns the counts per inch for each sensor
            CPI = [];
            if (obj.sensorInit == 1 && obj.sensorReading == 0)
                fwrite(obj.serialPort,obj.getParamByte,'int8');
                params = fread(obj.serialPort,2,'int8');
                CPI = params(1);
            end
        end
        function [LD] = GetLD(obj)
            % MotionReader GetLD
            % Returns the maximum distance between surface and sensors
            LD = [];
            if (obj.sensorInit == 1 && obj.sensorReading == 0)
                fwrite(obj.serialPort,obj.getParamByte,'int8');
                params = fread(obj.serialPort,2,'int8');
                LD = params(2);
            end
        end
        function SetParams(obj,params)
            % MotionReader SetParams
            % Set the CPI and LD parameters for each sensor:
            % params[1] = sensor 1 CPI
            % params[2] = sensor 1 LD
            if (length(params) ~=2)
                display('Params should have length 2')
                return;
            end
            if (obj.sensorInit == 1 && obj.sensorReading == 0)
                params = [4 params];
                fwrite(obj.serialPort, params, 'int8');
                
                obj.xScaling = 2.54/(params(1)*200);
                obj.yScaling = 2.54/(params(2)*200);
                display('Sensor parameters are set.');
            end
        end
        
        % Reading motion data
        function StartReading(obj)
            % MotionReader StartReading
            % Signal that the sensor should start collecting data
            if (obj.sensorInit == 1 && obj.sensorReading == 0)
                fwrite(obj.serialPort,obj.startReadingByte,'int8');
                obj.sensorReading = 1;
                display('Started reading from motion sensor');
            end
        end
        
        function StopReading(obj)
            % MotionReader StopReader
            % Signal that the sensor can stop collecting data
            if (obj.sensorInit == 1 && obj.sensorReading == 1)
                fwrite(obj.serialPort, obj.startReadingByte, 'int8');
                obj.sensorReading = 0;
                display('Stopped reading from motion sensor');
            end
        end
        
        function [x,y] = ReadSensor(obj)
            % MotionReader ReadSensor
            % Collects the x and y ticks that have been collected since the
            % previous read operation
            if (obj.sensorReading == 1)
               fwrite(obj.serialPort, obj.getDataByte, 'int8');
               data = fread(obj.serialPort,2,'int16');
            else
                data =[0 0];
            end
            x = data(1);%*obj.xScaling;
            y = data(2);%*obj.yScaling;
        end
    end   
end

