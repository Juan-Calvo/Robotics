%****************************************************************
%*   Inverse Kinematics 5-DOF robotic arm                       *
%*   2018/2019                                                  *
%****************************************************************
% Inverse Kinematics Class:
%   InvKinematics(mx,my,mz)
% Object parameters:
%   -mx:   X corordinate of the point
%   -my:   Y corordinate of the point
%   -mz:   Z corordinate of the point


classdef InvKinematics    
    properties (GetAccess=private, SetAccess=private)
        l1=233;
        l2=223;
        l3=223;
    end
    
    properties
        mx;
        my;
        mz;
        
        theta1;
        theta2;
        theta3;
        workspace_ok
    end
    
    methods
        function this=InvKinematics(mx,my,mz)
            this.mx=mx;
            this.my=my;
            this.mz=mz;
            for i=1:length(this.mx)
                this.theta1(i)=atan2d(this.my(i),this.mx(i));
                cosq3=((this.mx(i)^2)+(this.my(i)^2)+((this.mz(i)-this.l1)^2)-(this.l2^2)-((this.l3)^2))/(2*this.l2*this.l3);
                this.theta3(i)=atan2d(-sqrt(1-(cosq3)^2),cosq3);
                beta=atan2d(this.mz(i)-this.l1,-sqrt((this.mx(i)^2)+(this.my(i)^2)));
                alpha=atan2d(this.l3*sind(this.theta3(i)),this.l2+this.l3*cosd(this.theta3(i)));
               this.theta2(i)=abs(abs(alpha+beta)-180);
                if (imag(this.theta2(i))~=0)
                    disp("error theta 2 imaginary")
                    break;
                end
                if (imag(this.theta3(i))~=0)
                    disp("error theta 3 imaginary")
                    break;
                end
                if (this.theta3(i)>=-125 && 125>=this.theta3(i))
                    this.workspace_ok=true;
                else
                    disp("Error workspace theta3")
                    this.workspace_ok=false;
                end
                if (this.theta2(i)>=-180 && 180>=this.theta2(i))
                     this.workspace_ok=true;
                else
                    disp("Error workspace theta2")
                    this.workspace_ok=false;
                end
            end   
        end
    end
end

  