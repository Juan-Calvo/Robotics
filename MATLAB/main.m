close all
clear all

%Cartesian points
q=[200 200 200 150 -200 -200  150 200 0   300 200 150 -200 -300;
   200 300 300 350  300  300  250 150 200 300 200 200  300  200;
   200 200 300 250  300  200   50 200 200 350 30 300  380   50];

%Initial conditions
v0init=[0,0,0];
v1end=[0,0,0];

%Boundary conditions
vmax=130;
amax=300;
jmax=700;
r = 30;


for i=1:length(q)-1
 if (i>length(q)-2)
     linear(:,i)=Planner(bezier(:,i-1).qk_dd,q(:,length(q)),(bezier(:,i-1).t5k/bezier(:,i-1).lambda),v1end,vmax,amax,jmax); 
 else
   bezier(:,i)=bezier_a(q(:,i),q(:,i+1),q(:, i+2),r,vmax,amax,jmax);
   if (i==1)
    linear(:,i)=Planner(q(:,i), bezier(:,i).qk_d,v0init, (bezier(:,i).t0k/bezier(:,i).lambda),vmax,amax,jmax);
   else
    linear(:,i)=Planner(bezier(:,i-1).qk_dd,  bezier(:,i).qk_d,(bezier(:,i-1).t5k/bezier(:,i-1).lambda),(bezier(:,i).t0k/bezier(:,i).lambda),vmax,amax,jmax);  
   end
 end
end
i=0;
cont_lin=0;
cont_bezier=0;
n=length(q);
segmx=[];
segmy=[];
segmz=[];
tim=[];
sum_tim=[];


velx=[];
vely=[];
velz=[];
sample_time=0.05;
for i=1:(2*n-3)
    if (mod(i,2)==1)
        cont_lin=cont_lin+1;
        for j=0:sample_time:1
            if (cont_lin>0)
                if (j>0)
                segmx=[segmx,Position(linear(cont_lin).SCurves(1),j)];
                segmy=[segmy,Position(linear(cont_lin).SCurves(2),j)];
                segmz=[segmz,Position(linear(cont_lin).SCurves(3),j)];
                
                velx=[velx,Velocity(linear(cont_lin).SCurves(1),j)];
                vely=[vely,Velocity(linear(cont_lin).SCurves(2),j)];
                velz=[velz,Velocity(linear(cont_lin).SCurves(3),j)];
                
                
                tim=[tim,linear(cont_lin).max_time*sample_time];
                end
            else
                segmx=[segmx,Position(linear(cont_lin).SCurves(1),j)];
                segmy=[segmy,Position(linear(cont_lin).SCurves(2),j)];
                segmz=[segmz,Position(linear(cont_lin).SCurves(3),j)];
                tim=[tim,linear(cont_lin).max_time*sample_time];
                
                
                velx=[velx,Velocity(linear(cont_lin).SCurves(1),j)];
                vely=[vely,Velocity(linear(cont_lin).SCurves(2),j)];
                velz=[velz,Velocity(linear(cont_lin).SCurves(3),j)];
            end
        end
        j=0;
    else
        cont_bezier=cont_bezier+1;
       for j=0:sample_time:1
            bezier_points=Position(bezier(cont_bezier),j);
            vel=Velocity(bezier(cont_bezier),j)*bezier(cont_bezier).lambd;
               if (j>0)
                   
                  velx=[velx,vel(1)];
                  vely=[vely,vel(1)]; 
                  velz=[velz,vel(1)];
                   
                    segmx=[segmx, bezier_points(1)];
                    segmy=[segmy, bezier_points(2)];
                    segmz=[segmz, bezier_points(3)];
                    tim=[tim,bezier(cont_bezier).lambd*sample_time];
               end
       end
               
       j=0;
    end
end

inv=InvKinematics(segmx,segmy,segmz);
inv.theta2=90-inv.theta2;
error_q1=0;
error_q2=0;
error_q3=0;
error_t=0;
for ind=1:length(segmx)-1
        
        steps_q1(ind)=(((inv.theta1(ind+1)-inv.theta1(ind)))*8000/90)+error_q1;
        steps_q2(ind)=(((inv.theta2(ind+1)-inv.theta2(ind)))*2600/90)+error_q2;
        steps_q3(ind)=(((inv.theta3(ind+1)-inv.theta3(ind)))*24000/90)+error_q3;
   
        time_ok(ind)= abs(tim(ind+1)-tim(ind))*1000000+error_t;
        
        error_t= time_ok(ind)-round(time_ok(ind));
        
        error_q1=steps_q1(ind)-round(steps_q1(ind));
        error_q2=steps_q2(ind)-round(steps_q2(ind));
        error_q3=steps_q3(ind)-round(steps_q3(ind));
   
        time_ok(ind)= round(time_ok(ind));
        steps_q1(ind)=round(steps_q1(ind));
        steps_q2(ind)=round(steps_q2(ind));
        steps_q3(ind)=round(steps_q3(ind));
        
        if abs(steps_q1(ind))<1
            steps_q1(ind)=0;
        end
        if abs(steps_q2(ind))<1
            steps_q2(ind)=0;
        end
        if abs(steps_q3(ind))<1
            steps_q3(ind)=0;
        end
end
steps_q1=[round(inv.theta1(1)*8000/90), steps_q1];
steps_q2=[round(inv.theta2(1)*2600/90), steps_q2];
steps_q3=[round((inv.theta3(1)*24000/90)), steps_q3];
getmin=max(max(abs(steps_q1),abs(steps_q2)),abs(steps_q3));
tim=[300,rdivide( tim(2:length(tim))*1000000,    getmin(2:length(getmin))    )];
time=round(tim);
X='x';
Y='y';
Z='z';
W='w';
U='u';
T='t';
uu='0';
ww='0';
fin=';';
i=0;
s1='y';
s2='o';
init=0;
plot3(segmx,segmy,segmz);
tcp=tcpip('0.0.0.0',8090,'NetworkRole', 'server');
fopen(tcp);
for i=1:length(time)
    xx=num2str(steps_q1(i));
    yy=num2str(steps_q2(i));
    zz=num2str(steps_q3(i));
    tt=num2str(time(i));
    gcode=[X xx Y yy Z zz W ww U uu T tt fin];
    fwrite(tcp, gcode,'char');
    o=fread(tcp,1,'schar');
    while(strcmp(o,s2))
    end
    o='a';
end

fclose(tcp);
delete(tcp);