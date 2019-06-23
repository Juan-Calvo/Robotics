%****************************************************************
%*   S-Curve Planner                                            *
%*   2018/2019                                                  *
%****************************************************************
%S-Curve Planner Class:
%    Planner(q0,q1,v0,v1,vmax,amax,jmax)
%Object parameters:
%   -q0:    First point
%   -q1:    Second point
%   -v0:    Initial velocity
%   -v1:    Final velocity
%   -vmax:  Maximum velocity
%   -amax:  Maximum acceleration
%   -jmax:  Maximum jerk


classdef Planner
    properties
        q0;
        q1;
        v0;
        v1;
        vmax;
        amax;
        jmax;
        SCurves;
        max_time;
    end
    methods
        function this=Planner(q0,q1,v0,v1,vmax,amax,jmax)
            this.q0=q0;
            this.q1=q1;
            this.v0=v0;
            this.v1=v1;
            this.vmax=vmax;
            this.amax=amax;
            this.jmax=jmax;
            time=[];
            this.SCurves=[];
            for i=1:length(this.q0)
                SCurvess(i)=SCurve_a(this.q0(i),this.q1(i),this.v0(i),this.v1(i),this.vmax,this.amax,this.jmax);
                time=[time,SCurvess(i).T];
            end
            time=max(time);
            this.max_time=time;
            for i=1:length(this.q0)
                dq=abs(this.q1(i)-this.q0(i));
                if (dq<0.01)
                    SCurvess(i)=SCurve_a(this.q0(i),this.q1(i),this.v0(i),this.v1(i),this.vmax,this.amax,this.jmax,0);
                else
                SCurvess(i)=SCurve_a(this.q0(i),this.q1(i),this.v0(i),this.v1(i),this.vmax,this.amax,this.jmax,this.max_time);
                end
                this.SCurves=[this.SCurves,SCurvess(i)];
            end            
            
            
        end
    end
end