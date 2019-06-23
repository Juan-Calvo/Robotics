%****************************************************************
%*   Bézier Curve algorithm of 5-th degree                      *
%*   2018/2019                                                  *
%****************************************************************
%Bézier Class:
%   OPTION A:
%       bezier_a(qk1,qk,qk2,r)
%   OPTION B:
%       bezier_a(qk1,qk,qk2,r,vmax,amax,jmax)
%
%Object parameters:
%   -qk1:   First point of interpolation
%   -qk:    Second point of interpolation
%   -qk2:   Third point of interpolation
%   -r:     Tolerance
%   -vmax:  Maximum velocity
%   -amax:  Maximum acceleration
%   -jmax:  Maximum jerk

classdef bezier_a
  
  properties
  qk_d;
  qk_dd;
  t0k;
  t5k;
  p0k;
  p1k;
  p2k;
  p3k;
  p4k;
  p5k;
  a0k;
  a1k;
  a2k;
  a3k;
  a4k;
  a5k;
  lambd;
  max_in;
  lambda;

  end
  properties (GetAccess=private, SetAccess=private)
  qk1;
  qk;
  qk2;
  r;
  vmax;
  amax;
  jmax;
  end
  methods
   function this=bezier_a(varargin)
      if nargin ==1
        if strcmp(varargin{1},"help")
            disp("bezier_a(qk1,qk,qk2,r,vmax,amax,jmax)");
            return;
        else
            disp("ERROR: For more information write: help(bezier_a)");
            return;
        end
      elseif((nargin>3) && (nargin<8) ) 
           this.qk1= varargin{1};
           this.qk= varargin{2};
           this.qk2= varargin{3};
           this.r= varargin{4};
           this.max_in=false;
           if((nargin>4) && (nargin<8) )
               this.vmax=varargin{5};
               this.amax=varargin{6};
               this.jmax=varargin{7};
               this.max_in=true;
           end
      end
       this=ControlPoints(this);
       this=TangentVectors(this);
       this=IntermediateControlPoints(this);
       this=PolinomialParameters(this);
       this=Reparametrization(this);
   end
   function this=ControlPoints(this) 
       this.qk_d=((this.qk1-this.qk)/(norm(this.qk1-this.qk)))*this.r+this.qk;
       this.qk_dd=((this.qk2-this.qk)/(norm(this.qk2-this.qk)))*this.r+this.qk;
   end
   function this=TangentVectors(this)
       this.t0k=(this.qk-this.qk_d)/this.r;
       this.t5k=(this.qk_dd-this.qk)/this.r;
   end
   function this=IntermediateControlPoints(this)
        this.p0k=this.qk_d;
      	this.p5k=this.qk_dd;
        a=256-49*(norm( this.t0k+this.t5k))^2;
        b=420*transpose(this.p5k-this.p0k)*( this.t0k+this.t5k);
        c=-900*(norm(this.p5k-this.p0k))^2;
        alpha=(-b+sqrt(((b)^2)-4*a*c))/(2*a);
        this.p1k=this.p0k+(1/5)*alpha*this.t0k;
        this.p2k=2*this.p1k-this.p0k;
        this.p4k=this.p5k-(1/5)*alpha*this.t5k;
        this.p3k=2*this.p4k-this.p5k;
   end
   function this=PolinomialParameters(this)
       this.a0k=this.p0k;
        this.a1k=-5*this.p0k +5*this.p1k;
        this.a2k=10*this.p0k-20*this.p1k+10*this.p2k;
        this.a3k=-10*this.p0k+30*this.p1k-30*this.p2k+10*this.p3k;
        this.a4k=5*this.p0k-20*this.p1k+30*this.p2k-20*this.p3k+5*this.p4k; 
        this.a5k=-1*this.p0k+5*this.p1k-10*this.p2k+10*this.p3k-5*this.p4k+this.p5k;
   end
   function this=Reparametrization(this)
     this.lambd=5*(norm(this.p1k-this.p0k));
     if (this.max_in)
         max_vu=getMaxVel(this);
         lmbd_vel=this.vmax/max_vu;
         max_au=getMaxAcc(this);
         lmbd_acc=sqrt(this.amax/max_au);
         max_ju=getMaxJerk(this);
         lmbd_jrk=((this.jmax/max_ju)^(1/3));
         this.lambda=(1/min([(lmbd_vel),(lmbd_acc),(lmbd_jrk)]));
     else
         this.lambda=1;
     end
     this.lambd=this.lambda*this.lambd;
   end
   function [pos]=Position(this,u)
       [pos]=this.a0k + this.a1k*u + this.a2k*u^2 + this.a3k*u^3 + this.a4k*u^4 + this.a5k*u^5;
   end
   function [vel]=Velocity(this,u)
       ct=this.lambd;
       [vel]=((this.a1k + 2*this.a2k*u + 3*this.a3k*(u^2) + 4*this.a4k*(u^3) + 5*this.a5k*u^4)/ct);
   end
   function [acc]=Acceleration(this,u)
       ct=this.lambd^2;
       [acc]=((2*this.a2k + 6*this.a3k*u + 12*this.a4k*u^2 + 20*this.a5k*u^3)/ct);
   end
   function [j]=Jerk(this,u)
       ct=this.lambd^3;
       [j]=((6*this.a3k + 24*this.a4k*u + 60*this.a5k*u^2)/ct);
   end
   function [v]=getMaxVel(this)
        a=20*this.a5k;
        b=12*this.a4k;
        c=6*this.a3k;
        d=2*this.a2k;
        v_max=0;
        for w=1:length(this.a2k)
            p=[a(w),b(w),c(w),d(w)];
            times=roots(p);
            times=round(times*100000)/100000;
            t1=times(imag(times)==0);
            for i=1:length(t1)
                t=t1(i);
                if ((t>=0) && (t<=1))
                    result=abs(Velocity(this,t));
                    for j=1:length(result)
                        disp("aqui")
                        v_max=max([v_max,result(j)]);
                    end
                end
            end
        end
        if (v_max == 0)
            v0=max(abs(Velocity(this,0)));
            v1=max(abs(Velocity(this,1)));
            v_max=max(v0,v1);
        end
        [v]=v_max;
   end
   function [accel]=getMaxAcc(this)
        a=60*this.a5k;
        b=24*this.a4k;
        c=6*this.a3k;
        a_max=0;
        for w=1:length(this.a2k)
            p=[a(w),b(w),c(w)];
            times=roots(p);
            times=round(times*100000)/100000;
            t1=times(imag(times)==0);

            for i=1:length(t1)
                t=t1(i);
                if ((t>0) && (t<=1))
                    result=abs(Acceleration(this,t));
                    for j=1:length(result)
                        a_max=max([a_max,result(j)]);
                    end
                end
            end
        end
        if (a_max == 0)
            a0=max(abs(Acceleration(this,0)));
            a1=max(abs(Acceleration(this,1)));
            a_max=max(a0,a1);
        end
        [accel]=a_max;
   end
   function [jrk]=getMaxJerk(this)
        t1=-1*((24*this.a4k)/(120*this.a5k));
        jrk_max=0;
        for i=1:length(t1)
            t=t1(i);
            if ((t>0) && (t<=1))
                result=abs(Jerk(this,t));
                for j=1:length(result)
                    jrk_max=max([jrk_max,result(j)]);
                end
            end
        end
        if (jrk_max == 0)
            j0=max(abs(Jerk(this,0)));
            j1=max(abs(Jerk(this,1)));
            jrk_max=max(j0,j1);
        end
        [jrk]=jrk_max;
   end
  end
end