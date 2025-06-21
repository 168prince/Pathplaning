function parent=Astar()
Xo=[2 2];%起点位置
xTarget=9;%目标点x坐标
yTarget=9;%目标点y坐标
m_Target=[xTarget,yTarget];%目标点
J=200;%循环迭代次数
Xj=Xo;%j=1循环初始，将机器人的起始坐标赋给Xj
MAX_X=10;%地图X边界
MAX_Y=10;%地图Y边界
MAP=2*(ones(MAX_X,MAX_Y));%将所有点索值引初始化为2，区分路径点、一般点、障碍点
%选择机器人起点位置
xval=2;
yval=2;
xStart=xval;
yStart=yval;
xNode=xval;
yNode=yval;
path_cost=0;%初始代价为0
OPEN_COUNT=1;
MAP(xval,yval)=1;%机器人起点位置索引值初始化为1
MAP(1,4)=-1;%设置障碍索引值初始化为-1
MAP(1,5)=-1;   
MAP(1,9)=-1;
MAP(1,10)=-1;
MAP(2,4)=-1;
MAP(2,5)=-1;
MAP(2,9)=-1;
MAP(2,10)=-1;
MAP(3,5)=-1;
MAP(3,6)=-1;
MAP(3,9)=-1;
MAP(3,10)=-1;
MAP(4,2)=-1;
MAP(4,3)=-1;
MAP(4,5)=-1;
MAP(4,6)=-1;
MAP(5,2)=-1;
MAP(5,3)=-1;
MAP(6,6)=-1;
MAP(8,2)=-1;
MAP(8,8)=-1;
MAP(9,5)=-1;

%%%%%%%%%%%%%%%开始计算%%%%%%%%%%%%%%%%
OPEN=[];%用于存放可能路径点和代价函数值等
CLOSED=[];
j=0;
k=1;%存储障碍索引
for i=1:MAX_X
    for j=1:MAX_Y
        if(MAP(i,j) == -1)
            CLOSED(k,1)=i; 
            CLOSED(k,2)=j; 
            k=k+1;
        end
    end
end
CLOSED_COUNT=size(CLOSED,1);%计算有多少个障碍
goal_distance=distance(xNode,yNode,xTarget,yTarget);
OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,path_cost,...
    goal_distance,goal_distance);
parent(1,1)=xval;
parent(1,2)=yval;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for step=2:J%循环开始
    Goal(step,1)=m_Target(1);
    Goal(step,2)=m_Target(2);
    exp_array=expand_array(xNode,yNode,path_cost,xTarget,yTarget,CLOSED,...
        MAX_X,MAX_Y);
    exp_count=size(exp_array,1);

 for i=1:exp_count
    %判断是否停止循环
    if OPEN(OPEN_COUNT,2)==xTarget&&OPEN(OPEN_COUNT,3)==yTarget||...
            goal_distance<sqrt(2)
        OPEN(:,2)=xTarget;
        OPEN(:,3)=yTarget;%% 当前点是目的地的话就把目的地写入open
    else
        %OPEN加入下一组值
        OPEN_COUNT=OPEN_COUNT+1;
        OPEN(OPEN_COUNT,:)=insert_open(exp_array(i,1),exp_array(i,2),...
        xNode,yNode,exp_array(i,3),exp_array(i,4),exp_array(i,5));
    end;
 end
 best_num=2;
 L=size(OPEN,1);
 for OPEN_COUNT=2:1:L
     %比较估价函数大小
     if OPEN(OPEN_COUNT,8)<OPEN(best_num,8)
         best_num=OPEN_COUNT;    
     end
 end
 OPEN_COUNT=1;%清空OPEN 
 OPEN(OPEN_COUNT,:)=OPEN(best_num,:);
 %选定下一步
 parent(step,1)=OPEN(best_num,2);
 parent(step,2)=OPEN(best_num,3);
 xNode=parent(step,1);
 yNode=parent(step,2);
  goal_distance=distance(xNode,yNode,xTarget,yTarget); 
end











