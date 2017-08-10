classdef force_sensor
	% Class for reading ros topics from a force sensor

	% only this class methods can view/modify
    properties (SetAccess = private)
        % subscribers
        force_sensor_subscriber
    end

    methods
    	function self = force_sensor(topic)
    		% ----------- subscribers
            self.force_sensor_subscriber = ...
                rossubscriber(topic, rostype.geometry_msgs_WrenchStamped);
        end
        
        function vector = ros_wrench_to_vector(~, wrench)
           % convert idiotic ROS message type to a single vector
           vector = [wrench.Force.X,  wrench.Force.Y,  wrench.Force.Z, ...
                     wrench.Torque.X, wrench.Torque.Y, wrench.Torque.Z];
        end
        
        function seconds = ros_time_to_secs(~, stamp)
            % Convert awkward rostime into a single double
            seconds = double(stamp.Sec) + double(stamp.Nsec) * 10^-9;
        end
        
        function [wrench, timestamp] = get_wrench_current(self)
            % Accessor used to retrieve the last measured cartesian wrench
            wrench = self.ros_wrench_to_vector(self.force_sensor_subscriber.LatestMessage.Wrench);
            timestamp = self.ros_time_to_secs(self.force_sensor_subscriber.LatestMessage.Header.Stamp);
        end
    end

end