%****************************************************************
%*   S-Curve algorithm                                          *
%*   2018/2019                                                  *
%****************************************************************
%S-Curve Class:
%   OPTION A:
%       SCurve_a(q0,q1,v0,v1,vmax,amax, jmax)
%   OPTION B:
%       SCurve_a(q0,q1,v0,v1,vmax,amax, jmax, time)
%
%Object parameters:
%   -q0:    First point
%   -q1:    Second point
%   -v0:    Initial velocity
%   -v1:    Final velocity
%   -vmax:  Maximum velocity
%   -amax:  Maximum acceleration
%   -jmax:  Maximum jerk
%   -time:  Time to reach S-Curve profile
classdef SCurve_a 
    properties 
    sigma;vmin;amin;jmin;Tj1;Tj2;Tv;Ta;Td;Tj;alima;alimd;vlim;T;
            q0;
        q1;
    end
    properties (GetAccess=private, SetAccess=private)

        v0;
        v1;
        vmax;
        amax;
        jmax;
        calc_time;
    end
    methods
        function this=SCurve_a(varargin)
            if nargin ==1
                if strcmp(varargin{1},"help")
                        disp("SCurve_a(q0,q1,v0,v1,vmax,amax, jmax)");
                        return;
                else
                     disp("ERROR: For more information write: Prueva(""help"")");
                     return;
                end
            elseif((nargin>6) && (nargin<9) )
                this.q0 = varargin{1};
                this.q1 = varargin{2};
                this.v0 = varargin{3};
                this.v1 = varargin{4};
                this.vmax = varargin{5};
                this.amax = varargin{6};
                this.jmax = varargin{7};
                this.calc_time=false;
                if(nargin == 8)
                    this.T = varargin{8};
                    this.calc_time=true;
                end
            else
                disp("ERROR: Too many arguments");
                return;
            end
            this=check_velocities(this);
            check=check_is_feasable(this);
            if (check==false)
             disp('Error, is not feasable')
             return;
            end
            if(this.T==0)
                this.vmax=0;
                this.vlim=0;
                this.Td=0;
                this.Ta=0;
                this.Tv=0;
                this.T=0;
                this.Tj=0;
                this.Tj1=0;
                this.Tj2=0;
                this.alima=0;
                this.alimd=0;
                this.sigma=1;
            else
                this=sing_change(this);
                this=scurve_parameters(this);
                this=calc_max_param(this);
            end
        end
        function this=check_velocities(this)
            if this.vmax<abs(this.v0)
                this.v0=sign(this.v0)*this.vmax;
            end
            if this.vmax<abs(this.v1)
                this.v1=sign(this.v1)*this.vmax;
            end
        end
        
        function [bool]=check_is_feasable(this)
            dq=abs(this.q1-this.q0);
            dv=abs(this.v1-this.v0);
            time_set_velocity=sqrt((dv)/this.jmax);
            time_reach_acc=this.amax/this.jmax;
            this.Tj=min(time_set_velocity,time_reach_acc );
            if (this.Tj<time_reach_acc)
                if (dq<this.Tj*(this.v0+this.v1))
                    bool=false;
                else
                     bool=true;
                end
            elseif (this.Tj==time_reach_acc)
                 if (dq < (0.5 *(this.v0+this.v1)*(this.Tj+dv/this.amax)))
                    bool=false;
                 else
                     bool=true;
                 end
            else
                    bool=false;
            end     
        end
        
        function this=sing_change(this)
            this.sigma=sign(this.q1-this.q0);
            this.q0 = this.sigma*this.q0;
            this.q1 = this.sigma*this.q1;
            this.v0 = this.sigma*this.v0;
            this.v1 = this.sigma*this.v1;
            sigm1 = (this.sigma+1)/2;
            sigm2 = (this.sigma-1)/2;
            this.vmax = this.vmax*sigm1 - this.vmax*sigm2;
            this.amax = this.amax*sigm1 - this.amax*sigm2;
            this.jmax = this.jmax*sigm1 - this.jmax*sigm2;
            this.vmin = -this.vmax;
            this.amin = -this.amax;
            this.jmin = -this.jmax;
        end
        
        function this=scurve_parameters(this)
            this=calc_max_time_acc(this);
            if (this.calc_time==true)
                [this,bool]=min_vel_fortime(this);
                if (bool==false)
                    this=min_acc_tv_negative(this);
                end
            else
%                 this=calc_max_time_acc(this);
                if (this.Tv<0)
                    this=min_acc(this);
                end
            end
        end
        
        function this=calc_max_time_acc(this)
            if ((this.vmax-this.v0)*this.jmax < this.amax*this.amax) 
                this.Tj1 = sqrt((this.vmax-this.v0)/this.jmax);
                this.Ta  = 2*this.Tj1; 
            else 
                this.Tj1 = this.amax/this.jmax;
                this.Ta  = this.Tj1 + (this.vmax-this.v0)/this.amax;
            end
            if ((this.vmax-this.v1)*this.jmax < this.amax*this.amax)
               this.Tj2 = sqrt((this.vmax-this.v1)/this.jmax);
               this.Td  = 2*this.Tj2;       
            else 
                this.Tj2 = this.amax/this.jmax;
                this.Td  = this.Tj2 + (this.vmax-this.v1)/this.amax;
            end
            this.Tv = (this.q1-this.q0)/this.vmax - this.Ta*(1+this.v0/this.vmax)/2 - this.Td*(1+this.v1/this.vmax)/2;
        end
        function this = min_acc(this)
           it = 0;
           l = 0.99;
           max_iter = 200;
           while ((it < max_iter) && this.amax >= 0.01) 
                this = vmax_not_reach(this);
                if ((this.Ta < 2*this.Tj1) || (this.Td < 2*this.Tj2)) 
                    this.amax =this.amax* l;
                else
                    break;
                end
           end
        end
        function this = vmax_not_reach(this)
            this.Tj1 = this.amax/this.jmax;
            this.Tj2 = this.Tj1;
            this.Tj = this.Tj2;
            amax_squared = this.amax*this.amax;
            sqrt_delta = sqrt( ((amax_squared*amax_squared)/(this.jmax*this.jmax)) + 2*(this.v0*this.v0 + this.v1*this.v1)...
                + this.amax*(4*(this.q1-this.q0) - 2*this.amax*(this.v0+this.v1)/this.jmax));
            this.Ta = (amax_squared/this.jmax - 2*this.v0 + sqrt_delta)/(2*this.amax);
            this.Td = (amax_squared/this.jmax - 2*this.v1 + sqrt_delta)/(2*this.amax);
            this.Tv = 0;
            if (this.Ta<0)
                this=calc_no_acc(this);
            end
            if(this.Td<0)
                this=calc_no_decel(this);
            end
        end
        function this=calc_no_acc(this)
            this.Td = 2*((this.q1-this.q0)/(this.v1+this.v0));
            this.Tj2 = (this.jmax*(this.q1-this.q0)-sqrt(this.jmax*(this.jmax*((this.q1-this.q0)^2)+((this.v1+this.v0)^2)*(this.v1-this.v0...
                ))))/(this.jmax*(this.v1+this.v0));
            this.Ta=0;
            this.Tj1=0;
        end
        function this=calc_no_decel(this)
            this.Ta = 2*((this.q1-this.q0)/(this.v1+this.v0));
            this.Tj1 = (this.jmax*(this.q1-this.q0)-sqrt(this.jmax*(this.jmax*((this.q1-this.q0)^2)-((this.v1+this.v0)^2)*(this.v1-this.v0...
                ))))/(this.jmax*(this.v1+this.v0));
            this.Td=0;
            this.Tj2=0;
        end
        function this=calc_max_param(this)
            this.alima = this.jmax*this.Tj1;
            this.alimd = -this.jmax*this.Tj2;
            this.vmax = this.v0 + (this.Ta-this.Tj1)*this.alima;
            this.vlim = this.vmax;
            this.T= this.Ta+this.Tv+this.Td;
        
        end
        
        function [this,bool]=min_vel_fortime(this)
            this=calc_max_time_acc(this);
            Tpresent=this.Ta+this.Tv+this.Td;
            if(abs(this.T-Tpresent)<0.01)
                disp("T iguales")
                [bool]=true;
                return;
            end
            if(this.Tv<0)
                disp("Tv<0")
                [bool]=false;
                return;
            end
            vel_min=max(this.v1,this.v0);
            vel_max = this.vmax;
            lastav=[vel_min,vel_max];
            average=((vel_min+vel_max)/2);
            this.vmax=average;
            this=calc_max_time_acc(this);
            Tpresent=this.Ta+this.Tv+this.Td;
            error=abs(this.T-Tpresent);
            while ((error)>0.01)
                if (Tpresent<this.T)
                   lastav=[lastav(1),average];
                   average=((lastav(1)+lastav(2))/2);
                   if (average< vel_min+0.01)
                       [this,bool1]=min_acc_fortime(this);
                       [bool]=bool1;
                       
                       return;
                   end
                end
                if (Tpresent>this.T)
                    lastav=[average,lastav(2)];
                    average=((lastav(1)+lastav(2))/2);
                    
                    if average==this.vmax
                        [this,bool1]=min_acc_fortime(this);
                        [bool]=bool1;
                        return;
                    end
                end
                this.vmax=average;
                this.vmax=average;
                this=calc_max_time_acc(this);
                Tpresent=this.Ta+this.Tv+this.Td;
                error=abs(this.T-Tpresent);
            end
            if error<0.01
                [bool]=true;
                return;
            else
                [bool]=false;
                return;
            end
            
        end
        
        function [this,bool]=min_acc_fortime(this)
            a_min=0.01;
            a_max = this.amax;
            lastav=[a_min,a_max];
            average=((a_min+a_max)/2);
            this.amax=average;
            this=calc_max_time_acc(this);
            Tpresent=this.Ta+this.Tv+this.Td;
            error=abs(this.T-Tpresent);
            while ((error)>0.01)
                if (Tpresent<this.T)
                   lastav=[lastav(1),average];
                   average=((lastav(1)+lastav(2))/2);
                   if (average< a_min+0.01)
                       this.amax=a_max;
                       [bool]=false;
                       return;
                   end
                end
                if (Tpresent>this.T)
                    lastav=[average,lastav(2)];
                    average=((lastav(1)+lastav(2))/2);
                    
                    if average==this.amax
                        this.amax=a_max;
                        [bool]=false;
                        return;
                    end
                end
                this.amax=average;
                this=calc_max_time_acc(this);
                Tpresent=this.Ta+this.Tv+this.Td;
                error=abs(this.T-Tpresent);
            end
            if error<0.01
% % % % % % % % % % %                 modificado: [bool]=true;
                [bool]=false;
                return;
            else
                [bool]=false;
                return;
            end 
        end

        
                
        function this=min_acc_tv_negative(this)
            a_min=0.01;
            a_max = this.amax;
            lastav=[a_min,a_max];
            average=((a_min+a_max)/2);
            this.amax=average;
            this = vmax_not_reach(this);
            Tpresent=this.Ta+this.Tv+this.Td;
            error=abs(this.T-Tpresent);
            while ((error)>0.01)
                if (Tpresent<this.T)
                   lastav=[lastav(1),average];
                   average=((lastav(1)+lastav(2))/2);
                   if (average< a_min+0.01)
                       this.amax=a_max;
                       return;
                   end
                end
                if (Tpresent>this.T)
                    lastav=[average,lastav(2)];
                    average=((lastav(1)+lastav(2))/2);
                    
                    if average==this.amax
                        this.amax=a_max;
                        return;
                    end
                end
                this.amax=average;
                this = vmax_not_reach(this);
                Tpresent=this.Ta+this.Tv+this.Td;
                error=abs(this.T-Tpresent);
            end
            if error<0.01
                return;
            else
                return;
            end 
        end    
        function [pos]=Position(this,t)
            t=t*this.T;
             if (t<this.Tj1)
              [pos]=this.sigma*(this.q0+this.v0*t+this.jmax*t^3/6);
            elseif (t<(this.Ta-this.Tj1))
              [pos]=this.sigma*(this.q0+this.v0*t+(3*t.^2-3*this.Tj1*t+this.Tj1^2)*(this.alima/6));
            elseif (t<this.Ta)
              [pos]=this.sigma*(this.q0+(this.vlim+this.v0)*(this.Ta/2)-this.vlim*(this.Ta-t)-this.jmin*(this.Ta-t).^3/6);
            elseif (t<(this.Ta+this.Tv))
                [pos]=this.sigma*(this.q0+(this.vlim+this.v0)*(this.Ta/2)+this.vlim*(t-this.Ta));
            elseif (t<(this.T-this.Td+this.Tj2))
                [pos]=this.sigma*(this.q1-(this.vlim+this.v1)*(this.Td/2)+this.vlim*(t-this.T+this.Td)-this.jmax*((t-this.T+this.Td)^3/6));
            elseif (t<(this.T-this.Tj2))
                [pos]=this.sigma*(this.q1-(this.vlim+this.v1)*(this.Td/2)+this.vlim*(t-this.T+this.Td)+(this.alimd/6)*(3*(t-this.T+this.Td)^2-3*this.Tj2*(t-this.T+this.Td)+this.Tj2^2));
            elseif (t<=this.T)
                [pos]=this.sigma*(this.q1-this.v1*(this.T-t)-this.jmax*((this.T-t).^3)/6);
            else
                [pos]=this.sigma*this.q0;
            end
        end
        function [vel]=Velocity(this,t)
            t=t*this.T;
            if (t<this.Tj1)
                [vel]=this.sigma*(this.v0+this.jmax*t^2/2);
            elseif (t<(this.Ta-this.Tj1))
                [vel]=this.sigma*(this.v0+this.alima*(t-(this.Tj1/2)));
            elseif (t<this.Ta)
                [vel]=this.sigma*(this.vlim+this.jmin*((this.Ta-t).^2/2));
            elseif (t<(this.Ta+this.Tv))
                [vel]=this.sigma*this.vlim;
            elseif (t<(this.T-this.Td+this.Tj2))
                [vel]=this.sigma*(this.vlim-this.jmax*((t-this.T+this.Td)^2/2));
            elseif (t<(this.T-this.Tj2))
                [vel]=this.sigma*(this.vlim+this.alimd*(t-this.T+this.Td-(this.Tj2/2)));
            elseif (t<=this.T)
                [vel]=this.sigma*(this.v1+this.jmax*((this.T-t).^2)/2);
            else
                [vel]=this.sigma*this.v1;
            end
        end
        function [acc]=Acceleration(this,t)
            t=t*this.T;
            if (t<this.Tj1)
                [acc]=this.sigma*(this.jmax*t);  
            elseif (t<(this.Ta-this.Tj1))
                [acc]=this.sigma*this.alima;
            elseif (t<this.Ta)
                [acc]=this.sigma*(-this.jmin*(this.Ta-t));
            elseif (t<(this.Ta+this.Tv))
                [acc]=0;
            elseif (t<(this.T-this.Td+this.Tj2))
                [acc]=this.sigma*(-this.jmax*(t-this.T+this.Td));
            elseif (t<(this.T-this.Tj2))
                [acc]=this.sigma*this.alimd;
            elseif (t<=this.T)
                [acc]=this.sigma*(-this.jmax*(this.T-t));
            else
                [acc]=a(this.T);
            end
        end
        function [jerk]=Jerk(this,t)
            t=t*this.T;
            if (t<this.Tj1)
              [jerk]=this.sigma*this.jmax;
            elseif (t<(this.Ta-this.Tj1))
              [jerk]=this.sigma*0;
            elseif (t<this.Ta)
              [jerk]=this.sigma*this.jmin;
            elseif (t<(this.Ta+this.Tv))
              [jerk]=this.sigma*0;
            elseif (t<(this.T-this.Td+this.Tj2))
              [jerk]=this.sigma*this.jmin;
            elseif (t<(this.T-this.Tj2))
              [jerk]=this.sigma*0;
            elseif (t<=this.T)
              [jerk]=this.sigma*this.jmax;
            end
        end
    end
end