clear all
clc
close all

%prediction step
N=40;
%time sample
delt = 200e-3;
%road information
width=3.5;
L=4.57;
%car initialization
X=[0; width/2; 0; 0]; % x; y; (psi(body steering angle;  v
beta= atan2(L/2*tan(X(4)/L),1); % (beta) body slip angle;
delta= 0;  %(del)heading angle;
U=zeros(2,N); %del_dot v_dot
%weight matrix
P0=eye(3);
Q=eye(3);
R=eye(2);
%y=Cx
C=[1 0 0 0; 0 1 0 0; 0 0 0 1];
C_star=[1 0 0 0 ; 0 1 0 0; 0 0 0 0];
Y=C*X;

%yd : desired waypoint
Yd = [100; 10; 5];
Y_=Yd-Y;
%constraint value
umax = [5 * pi/180; 1.5]; % del_dot v_dot
Rc = 1; %moving obstacle safe distance
Ro = 1; %static obstacle safe distance
Ru = 1; %safe distance from the obstacles
move_obs_num=3;
sta_obs_num=1;
lamda=zeros(4,N+1); % 4 is state num
%weight matrix
P0=0.5*eye(3);
Q=eye(3)*10;
R=eye(2);

E_=200e-3;%cost function critical point
%car information
hold on
ego_car_img=fill([Y(1,1) Y(1,1)-L Y(1,1)-L Y(1,1)], [[1.8/2, 1.8/2, -1.8/2, -1.8/2] + Y(2,1)],'r');
[front_car, front_car_img] = front_cardef(width);
[adj_behind_car, adj_behind_car_img] = adj_behind_car_cardef(width);
[adj_front_car, adj_front_car_img]=adj_front_car_cardef(width);

qv(1,:)=abs(U(2,:))-umax(2);
qdel(1,:)=abs(U(1,:))-umax(1);
Sj(1,1)=Rc-norm(Y(1:2,1)-[front_car.x; front_car.y]);
Sj(2,1)=Rc-norm(Y(1:2,1)-[adj_behind_car.x; adj_behind_car.y]);
Sj(3,1)=Rc-norm(Y(1:2,1)-[adj_front_car.x; adj_front_car.y]);
Sl(1)=Ro+Ru-norm(Y(1:2,1)-[10; 0]); %To change the static obstacle position, i should change the second line of Slx function in Get_Lamda.m

%initial objective function
J=inf;
dJ=inf;
partial_H_u=[];
 while abs(dJ)>E_
     if exist('X(:,2)','var') == 1
         delta(1)=delta(2);
         beta(1)=beta(2);
         X(1,1)=X(1,2);
         X(2,1)=X(2,2);
         X(3,1)=X(3,2);
         X(4,1)=X(4,2);
         Y(:,1)=Y(:,2);
         Y_(:,1)=Y_(:,2);
         qv(1)=qv(2);
         qdel(1)=qdel(2);
         Sj(:,1)=Sj(:,2);
         Sl(1)=Sl(2);
     end
    for i=1:N %x(1) ->now x(N+1) ->after N step
        [X,beta,delta,qv,qdel,Sj,Sl,Y,Y_]=Get_Next_States(C,X,U,L,delt,beta,delta,i,umax,Rc,Ro,Ru,front_car,adj_behind_car,adj_front_car,Y,Y_,Yd,qv,qdel,Sj,Sl);
    end
    for i=N+1:-1:1 %lamda0(now) ~lamdaN
        lamda = Get_Lamda(L,X,beta,delta,lamda,N,Y_,P0,Q,C,i,move_obs_num,sta_obs_num,Sj,Sl,front_car,adj_behind_car,adj_front_car,C_star,delt);
    end
    for i=1:N
        partial_H_u=get_partial_H_u(U,R,lamda,i,qv,qdel,partial_H_u);
    end
    J_pre=J;
    J=compute_J(P0,Q,R,lamda,X,qv,qdel,Sj,Sl,Y_,U,delta,beta,move_obs_num,sta_obs_num,N,L);
%     dJ=comdpute_dJ()
    if J-J_pre<=0
        U=update_U(partial_H_u,delt,U);
    else
        delt=9/10*delt;
    end
    Y
    axis([Y(1,1)-10 Y(1,1)+10 Y(1,1)-10 Y(1,2)+10])
    front_car.x=front_car.v*delt+front_car.x;
    adj_front_car.x=adj_front_car.v*delt+adj_front_car.x;
    adj_behind_car.x=adj_behind_car.v*delt+adj_behind_car.x;
    set(ego_car_img,'Xdata',[Y(1,1) Y(1,1)-L Y(1,1)-L Y(1,1)],'Ydata',Y(2,1))
    set(front_car_img,'Xdata',front_car.x,'Ydata',front_car.y)
    set(adj_front_car_img,'Xdata',adj_front_car.x,'Ydata',adj_front_car.y)
    set(adj_behind_car_img,'Xdata',adj_behind_car.x,'Ydata',adj_behind_car.y)
    pause(1)
 end